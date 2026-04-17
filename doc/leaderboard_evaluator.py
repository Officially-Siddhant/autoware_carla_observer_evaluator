#!/usr/bin/env python
# Copyright (c) 2018-2019 Intel Corporation.
# authors: German Ros (german.ros@intel.com), Felipe Codevilla (felipe.alcm@gmail.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Challenge Evaluator Routes

Provisional code to evaluate Autonomous Agents for the CARLA Autonomous Driving challenge
"""
from __future__ import print_function

import traceback
import argparse
from argparse import RawTextHelpFormatter
import importlib
import os
from packaging.version import Version
import sys
import carla
import signal
from datetime import datetime
from pathlib import Path
import yaml

from srunner.scenariomanager.carla_data_provider import *
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from leaderboard.scenarios.scenario_manager import ScenarioManager
from leaderboard.scenarios.route_scenario import RouteScenario
from leaderboard.envs.sensor_interface import SensorConfigurationInvalid
from leaderboard.autoagents.agent_wrapper import AgentError, validate_sensor_configuration
from leaderboard.utils.statistics_manager import StatisticsManager, FAILURE_MESSAGES
from leaderboard.utils.route_indexer import RouteIndexer


sensors_to_icons = {
    "sensor.camera.rgb": "carla_camera",
    "sensor.lidar.ray_cast": "carla_lidar",
    "sensor.other.radar": "carla_radar",
    "sensor.other.gnss": "carla_gnss",
    "sensor.other.imu": "carla_imu",
    "sensor.opendrive_map": "carla_opendrive_map",
    "sensor.speedometer": "carla_speedometer",
}

class LeaderboardEvaluator(object):
    """
    Main class of the Leaderboard. Everything is handled from here,
    from parsing the given files, to preparing the simulation, to running the route.
    """

    # Tunable parameters
    client_timeout = 10.0  # in seconds
    frame_rate = 20.0      # in Hz

    def __init__(self,config_eval: dict, statistics_manager: StatisticsManager,):
        """Setup CARLA client and world and Setup ScenarioManger

        Args:
            config_eval (dict): Configuration for evaluating an agent
            statistics_manager (StatisticsManager): Carla Leaderboard
                Statistics Manager

        Raises:
            ImportError
        """
        self.world = None
        self.manager = None
        self.sensors = None
        self.sensors_initialized = False
        self.sensor_icons = []
        self.agent_instance = None
        self.route_scenario = None

        self.statistics_manager = statistics_manager

        # This is the ROS1 bridge server instance. This is not encapsulated inside the ROS1 agent because the same
        # instance is used on all the routes (i.e., the server is not restarted between routes). This is done
        # to avoid reconnection issues between the server and the roslibpy client.
        self._ros1_server = None

        # Setup the simulation
        self.client, self.client_timeout, self.traffic_manager = self._setup_simulation(config_eval)

        try:
            dist_version = importlib.metadata.version("carla")
        except importlib.metadata.PackageNotFoundError as exc:
            raise ImportError("CARLA package is not installed") from exc

        if dist_version != "leaderboard":
            if Version(dist_version) < Version("0.9.10"):
                raise ImportError(
                    f"CARLA version 0.9.10.1 or newer required. "
                    f"CARLA version found: {dist_version}",
                )

        # Load agent - this is the agent that is called in aw_e2e.py
        agent = config_eval["agent_setup"]["agent"]
        module_name = os.path.basename(agent).split(".")[0]
        sys.path.insert(
            0,
            os.path.dirname(agent),
        )
        self.module_agent = importlib.import_module(module_name)

        # Create the ScenarioManager
        self.manager = ScenarioManager(
            timeout=config_eval["general_parameters"]["timeout"],
            statistics_manager=self.statistics_manager,
            debug_mode=config_eval["general_parameters"]["debug"],
        )

        # Time control for summary purposes
        self._start_time = GameTime.get_time()
        self._end_time = None

        # Prepare the agent timer
        self._agent_watchdog = None
        signal.signal(signal.SIGINT, self._signal_handler)

        self._client_timed_out = False

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt.
        Either the agent initialization watchdog is triggered, or the runtime one at scenario manager
        """
        if self._agent_watchdog and not self._agent_watchdog.get_status():
            raise RuntimeError("Timeout: Agent took longer than {}s to setup".format(self.client_timeout))
        elif self.manager:
            self.manager.signal_handler(signum, frame)

    def __del__(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """
        if hasattr(self, 'manager') and self.manager:
            del self.manager
        if hasattr(self, 'world') and self.world:
            del self.world

    def _get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occurred, True otherwise
        """
        if self._agent_watchdog:
            return self._agent_watchdog.get_status()
        return False

    def _cleanup(self):
        """
        Remove and destroy all actors
        """
        CarlaDataProvider.cleanup()

        if self._agent_watchdog:
            self._agent_watchdog.stop()

        try:
            if self.agent_instance:
                self.agent_instance.destroy()
                self.agent_instance = None
        except Exception as e:
            print("\n\033[91mFailed to stop the agent:")
            print(f"\n{traceback.format_exc()}\033[0m")

        if self.route_scenario:
            self.route_scenario.remove_all_actors()
            self.route_scenario = None
            if self.statistics_manager:
                self.statistics_manager.remove_scenario()

        if self.manager:
            self._client_timed_out = not self.manager.get_running_status()
            self.manager.cleanup()

        # Make sure no sensors are left streaming
        alive_sensors = self.world.get_actors().filter('*sensor*')
        for sensor in alive_sensors:
            sensor.stop()
            sensor.destroy()

    def _setup_simulation(self, config_eval):
        """
        Prepares the simulation by getting the client, and setting up the world and traffic manager settings
        """
        client = carla.Client(
            config_eval["general_parameters"]["host"], # localhost
            config_eval["general_parameters"]["port"], # 2000
        )

        config_timeout = config_eval["general_parameters"]["timeout"] # we can change this later on
        if config_timeout:
            client_timeout = config_timeout
        else:
            client_timeout = self.client_timeout

        settings = carla.WorldSettings(
            synchronous_mode = True,
            fixed_delta_seconds = 1.0 / self.frame_rate, # dt = 0.05s
            deterministic_ragdolls = True,
            spectator_as_ego = False
        )
        client.get_world().apply_settings(settings)

        traffic_manager = client.get_trafficmanager(config_eval["general_parameters"]["traffic-manager-port"],)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_hybrid_physics_mode(True)

        return client, client_timeout, traffic_manager

    def _reset_world_settings(self):
        """
        Changes the modified world settings back to asynchronous
        """
        # Has simulation failed?
        if self.world and self.manager and not self._client_timed_out:
            # Reset to asynchronous mode
            self.world.tick()  # TODO: Make sure all scenario actors have been destroyed
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            settings.deterministic_ragdolls = False
            settings.spectator_as_ego = True
            self.world.apply_settings(settings)

            # Make the TM back to async
            self.traffic_manager.set_synchronous_mode(False)
            self.traffic_manager.set_hybrid_physics_mode(False)

    def _load_and_wait_for_world(self, configuration, town):
        """
        Load a new CARLA world without changing the settings and provide data to CarlaDataProvider
        """
        current_map = self.client.get_world().get_map().name.split('/')[-1]
        if current_map == town:
            print(f"Map {town} already loaded, skipping reload.")
            self.world = self.client.get_world()
        else:
            self.world = self.client.load_world(town, reset_settings=False)

        # Large Map settings are always reset, for some reason
        settings = self.world.get_settings()
        settings.tile_stream_distance = 650
        settings.actor_active_distance = 650
        self.world.apply_settings(settings)

        self.world.reset_all_traffic_lights()
        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_traffic_manager_port(
            configuration["general_parameters"]["traffic-manager-port"],
        )
        CarlaDataProvider.set_world(self.world)

        # This must be here so that all route repetitions use the same 'unmodified' seed
        self.traffic_manager.set_random_device_seed(
            configuration["general_parameters"]["traffic-manager-seed"],
        )

        # Wait for the world to be ready
        self.world.tick()

        map_name = CarlaDataProvider.get_map().name.split("/")[-1]
        if map_name != town:
            raise Exception("The CARLA server uses the wrong map!"
                            " This scenario requires the use of map {}".format(town))

    def _register_statistics(self, route_index, entry_status, crash_message=""):
        """
        Computes and saves the route statistics
        """
        print("\033[1m> Registering the route statistics\033[0m")
        self.statistics_manager.save_entry_status(entry_status)
        current_record = self.statistics_manager.compute_route_statistics(
            route_index, self.manager.scenario_duration_system, self.manager.scenario_duration_game, crash_message
        )
        return current_record

    def _load_and_run_scenario(self, config_eval, config_scenario):
        """
        Load and run the scenario given by config.

        Depending on what code fails, the simulation will either stop the route and
        continue from the next one, or report a crash and stop.
        """
        crash_message = ""
        entry_status = "Started"

        print("\n\033[1m========= Preparing {} (repetition {}) =========\033[0m".format(config_scenario.name, config_scenario.repetition_index))

        # Prepare the statistics of the route
        route_name = f"{config_scenario.name}_rep{config_scenario.repetition_index}"
        self.statistics_manager.create_route_data(route_name, config_scenario.index)

        print("\033[1m> Loading the world\033[0m")

        # Load the world and the scenario
        debug = config_eval["general_parameters"]["debug"]
        try:
            self._load_and_wait_for_world(config_eval, config_scenario.town)
            self.route_scenario = RouteScenario(
                world=self.world,
                config=config_scenario,
                debug_mode=debug,
            )
            self.statistics_manager.set_scenario(self.route_scenario)

        except Exception:
            # The scenario is wrong -> set the execution to crashed and stop
            print("\n\033[91mThe scenario could not be loaded:")
            print(f"\n{traceback.format_exc()}\033[0m")

            entry_status, crash_message = FAILURE_MESSAGES["Simulation"]
            self._register_statistics(config_scenario.index, entry_status, crash_message)
            self._cleanup()
            return True

        print("\033[1m> Setting up the agent\033[0m")

        # Set up the user's agent, and the timer to avoid freezing the simulation
        timeout = config_eval["general_parameters"]["timeout"]
        try:
            self._agent_watchdog = Watchdog(timeout)
            self._agent_watchdog.start()
            agent_class_name = getattr(self.module_agent, 'get_entry_point')()
            agent_class_obj = getattr(self.module_agent, agent_class_name)
            route_string = (
                f"{agent_class_name}_"
                f"{Path(config_eval['simulation_setup']['routes']).stem}_"
                f"Route_{config_scenario.index}_"
                f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
            )

            # Start the ROS1 bridge server only for ROS1 based agents.
            if getattr(agent_class_obj, 'get_ros_version')() == 1 and self._ros1_server is None:
                from leaderboard.autoagents.ros1_agent import ROS1Server
                self._ros1_server = ROS1Server()
                self._ros1_server.start()

            self.agent_instance = agent_class_obj(config_eval["general_parameters"]["host"], config_eval["general_parameters"]["port"], config_eval["general_parameters"]["debug"])
            self.agent_instance.set_global_plan(self.route_scenario.gps_route, self.route_scenario.route)
            self.agent_instance.setup(config_eval["agent_setup"]["agent-config"])

            # Check and store the sensors
            if not self.sensors:
                self.sensors = self.agent_instance.sensors()
                track = self.agent_instance.track

                validate_sensor_configuration(self.sensors, track, config_eval["challenge"]["track"])

                self.sensor_icons = [sensors_to_icons[sensor['type']] for sensor in self.sensors]
                self.statistics_manager.save_sensors(self.sensor_icons)
                self.statistics_manager.write_statistics()

                self.sensors_initialized = True

            self._agent_watchdog.stop()
            self._agent_watchdog = None

        except SensorConfigurationInvalid as e:
            # The sensors are invalid -> set the execution to rejected and stop
            print("\n\033[91mThe sensor's configuration used is invalid:")
            print(f"{e}\033[0m\n")

            entry_status, crash_message = FAILURE_MESSAGES["Sensors"]
            self._register_statistics(config_scenario.index, entry_status, crash_message)
            self._cleanup()
            return True

        except Exception:
            # The agent setup has failed -> start the next route
            print("\n\033[91mCould not set up the required agent:")
            print(f"\n{traceback.format_exc()}\033[0m")

            entry_status, crash_message = FAILURE_MESSAGES["Agent_init"]
            results = self._register_statistics(config_scenario.index, entry_status, crash_message)
            self._cleanup()
            return False

        print("\033[1m> Running the route\033[0m")

        # Run the scenario
        record = config_eval["general_parameters"]["record"]
        try:
            # Load scenario and run it
            if record:
                self.client.start_recorder("{}/{}_rep{}.log".format(record, config_scenario.name, config_scenario.repetition_index))
            self.manager.load_scenario(self.route_scenario, self.agent_instance, config_scenario.index, config_scenario.repetition_index)
            self.manager.run_scenario()

        except AgentError:
            # The agent has failed -> stop the route
            print("\n\033[91mStopping the route, the agent has crashed:")
            print(f"\n{traceback.format_exc()}\033[0m")

            entry_status, crash_message = FAILURE_MESSAGES["Agent_runtime"]

        except Exception:
            print("\n\033[91mError during the simulation:")
            print(f"\n{traceback.format_exc()}\033[0m")

            entry_status, crash_message = FAILURE_MESSAGES["Simulation"]

        # Stop the scenario
        try:
            print("\033[1m> Stopping the route\033[0m")
            self.manager.stop_scenario()
            self._register_statistics(config_scenario.index, entry_status, crash_message)

            if record:
                self.client.stop_recorder()

            self._cleanup()

        except Exception:
            print("\n\033[91mFailed to stop the scenario, the statistics might be empty:")
            print(f"\n{traceback.format_exc()}\033[0m")

            _, crash_message = FAILURE_MESSAGES["Simulation"]

        # If the simulation crashed, stop the leaderboard, for the rest, move to the next route
        return crash_message == "Simulation crashed"

    def run(self, config_eval):
        """
        Run the challenge mode
        """
        route_indexer = RouteIndexer(config_eval["simulation_setup"]["routes"], config_eval["simulation_setup"]["repetitions"], config_eval["simulation_setup"]["routes-subset"])

        if config_eval["challenge"]["resume"]:
            resume = route_indexer.validate_and_resume(config_eval["challenge"]["checkpoint"])
        else:
            resume = False

        if resume:
            self.statistics_manager.add_file_records(config_eval["challenge"]["checkpoint"])
        else:
            self.statistics_manager.clear_records()
        self.statistics_manager.save_progress(route_indexer.index, route_indexer.total)
        self.statistics_manager.write_statistics()

        crashed = False
        while route_indexer.peek() and not crashed:

            # Run the scenario
            config_scenario = route_indexer.get_next_config()
            crashed = self._load_and_run_scenario(config_eval, config_scenario)

            # Save the progress and write the route statistics
            self.statistics_manager.save_progress(route_indexer.index, route_indexer.total)
            self.statistics_manager.write_statistics()

        # Shutdown ROS1 bridge server if necessary
        if self._ros1_server is not None:
            self._ros1_server.shutdown()

        # Go back to asynchronous mode
        self._reset_world_settings()

        if not crashed:
            # Save global statistics
            print("\033[1m> Registering the global statistics\033[0m")
            self.statistics_manager.compute_global_statistics()
            self.statistics_manager.validate_and_write_statistics(self.sensors_initialized, crashed)

        return crashed

def main():
    description = "CARLA AD Leaderboard Evaluation: evaluate your Agent in CARLA scenarios\n"

    # general parameters
    parser = argparse.ArgumentParser(description=description,formatter_class=RawTextHelpFormatter,)
    parser.add_argument(
        "--conf_file_path",
        default="config/config.yaml",
        help="Path to configuration file",
        type=str,
    )

    arguments = parser.parse_args()

    with open(
        Path(Path.cwd(), arguments.conf_file_path),
        encoding="utf-8",
    ) as f:
        config_eval = yaml.safe_load(f)["evaluation"]

    statistics_manager = StatisticsManager(config_eval["challenge"]["checkpoint"], config_eval["challenge"]["debug-checkpoint"])
    leaderboard_evaluator = LeaderboardEvaluator(config_eval, statistics_manager)
    crashed = leaderboard_evaluator.run(config_eval)

    del leaderboard_evaluator

    if crashed:
        sys.exit(-1)
    else:
        sys.exit(0)

if __name__ == '__main__':
    main()
