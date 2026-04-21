#!/usr/bin/env python
# Simplified CARLA vehicle spawner for motion planner testing.
# Spawns the Audi e-tron ego vehicle with its full sensor kit at a fixed
# spawn point and keeps the simulation ticking indefinitely, publishing
# all sensor data over ROS2 topics via the autoware_carla_cpp_bridge.
# No evaluation, no route completion checks, no scenario manager.
# Stop with Ctrl+C — all actors are cleaned up on exit.

from __future__ import print_function

import argparse
from argparse import RawTextHelpFormatter
import importlib
import os
import sys
import signal
import time
import traceback

import carla
import yaml
from pathlib import Path

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from leaderboard.autoagents.agent_wrapper import validate_sensor_configuration
from leaderboard.envs.sensor_interface import SensorConfigurationInvalid


class VehicleSpawner(object):
    """
    Spawns the ego vehicle with its full sensor kit and ticks the simulation
    indefinitely. Does not run any evaluation or scenario management.
    All sensors publish to ROS2 topics via the autoware_carla_cpp_bridge.
    """

    frame_rate = 20.0  # Hz — must match the bridge expectation

    def __init__(self, config_eval: dict):
        self.world = None
        self.ego_vehicle = None
        self.agent_instance = None
        self.sensors = None
        self._running = True

        # Connect to CARLA
        print("[Spawner] Connecting to CARLA...")
        self.client = carla.Client(
            config_eval["general_parameters"]["host"],
            config_eval["general_parameters"]["port"],
        )
        self.client.set_timeout(config_eval["general_parameters"]["timeout"])

        # Get the current world — do NOT reload the map
        self.world = self.client.get_world()
        current_map = self.world.get_map().name.split("/")[-1]
        print(f"[Spawner] Connected. Current map: {current_map}")

        # Set synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / self.frame_rate
        settings.spectator_as_ego = False
        self.world.apply_settings(settings)
        print(f"[Spawner] Synchronous mode enabled at {self.frame_rate}Hz")

        # Set up CarlaDataProvider (required by the agent)
        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_world(self.world)

        # Load the agent module
        agent_path = config_eval["agent_setup"]["agent"]
        module_name = os.path.basename(agent_path).split(".")[0]
        sys.path.insert(0, os.path.dirname(agent_path))
        self.module_agent = importlib.import_module(module_name)

        # Handle Ctrl+C gracefully
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print("\n[Spawner] Ctrl+C received — shutting down...")
        self._running = False

    def _get_spawn_point(self, config_eval):
        """
        Returns the first available spawn point from the map.
        Override this to use a specific location.
        """
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available in this map.")
        return spawn_points[0]

    def _spawn_ego_vehicle(self, config_eval):
        """
        Spawns the Audi e-tron ego vehicle and attaches all sensors
        defined by the agent's sensor() method.
        """
        print("[Spawner] Spawning ego vehicle...")

        # Get the agent class and instantiate it
        agent_class_name = getattr(self.module_agent, 'get_entry_point')()
        agent_class_obj = getattr(self.module_agent, agent_class_name)

        self.agent_instance = agent_class_obj(
            config_eval["general_parameters"]["host"],
            config_eval["general_parameters"]["port"],
            config_eval["general_parameters"]["debug"],
        )
        self.agent_instance.setup(config_eval["agent_setup"]["agent-config"])

        # Validate and retrieve the sensor configuration
        self.sensors = self.agent_instance.sensors()
        track = self.agent_instance.track
        validate_sensor_configuration(
            self.sensors, track, config_eval["challenge"]["track"]
        )

        print(f"[Spawner] Agent loaded: {agent_class_name}")
        print(f"[Spawner] Sensors: {[s['type'] for s in self.sensors]}")

        # Tick once to let everything initialize
        self.world.tick()
        print("[Spawner] Ego vehicle and sensors spawned successfully.")
        print("[Spawner] Sensor data is now publishing to ROS2 topics.")
        print("[Spawner] Press Ctrl+C to stop.\n")

    def _cleanup(self):
        """
        Destroys all spawned actors and resets world to asynchronous mode.
        """
        print("[Spawner] Cleaning up actors...")

        try:
            if self.agent_instance:
                self.agent_instance.destroy()
                self.agent_instance = None
        except Exception:
            print("[Spawner] Warning: failed to destroy agent instance.")
            print(traceback.format_exc())

        # Destroy any remaining sensors
        try:
            alive_sensors = self.world.get_actors().filter('*sensor*')
            for sensor in alive_sensors:
                sensor.stop()
                sensor.destroy()
            print(f"[Spawner] Destroyed {len(alive_sensors)} sensor(s).")
        except Exception:
            print("[Spawner] Warning: failed to destroy some sensors.")

        # Destroy remaining vehicles
        try:
            alive_vehicles = self.world.get_actors().filter('*vehicle*')
            for vehicle in alive_vehicles:
                vehicle.destroy()
            print(f"[Spawner] Destroyed {len(alive_vehicles)} vehicle(s).")
        except Exception:
            print("[Spawner] Warning: failed to destroy some vehicles.")

        # Reset to asynchronous mode
        try:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            settings.spectator_as_ego = True
            self.world.apply_settings(settings)
            print("[Spawner] World reset to asynchronous mode.")
        except Exception:
            print("[Spawner] Warning: failed to reset world settings.")

        CarlaDataProvider.cleanup()

    def run(self, config_eval):
        """
        Spawns the ego vehicle and ticks the simulation indefinitely.
        Prints tick count and game time every 100 ticks.
        """
        try:
            self._spawn_ego_vehicle(config_eval)
        except SensorConfigurationInvalid as e:
            print(f"\n[Spawner] Sensor configuration invalid: {e}")
            self._cleanup()
            return
        except Exception:
            print("\n[Spawner] Failed to spawn ego vehicle:")
            print(traceback.format_exc())
            self._cleanup()
            return

        tick_count = 0
        game_time = 0.0

        while self._running:
            self.world.tick()
            tick_count += 1
            game_time += 1.0 / self.frame_rate

            if tick_count % 100 == 0:
                print(
                    f"[Spawner] Tick {tick_count:6d} | "
                    f"Game time {game_time:8.2f}s | "
                    f"Running at {self.frame_rate}Hz"
                )

        self._cleanup()


def main():
    description = (
        "CARLA Vehicle Spawner for Motion Planner Testing\n"
        "Spawns the Audi e-tron ego vehicle with its full sensor kit.\n"
        "Publishes sensor data to ROS2 topics via autoware_carla_cpp_bridge.\n"
        "Does not run evaluation — use for motion planner development.\n"
    )

    parser = argparse.ArgumentParser(
        description=description,
        formatter_class=RawTextHelpFormatter,
    )
    parser.add_argument(
        "--conf_file_path",
        default="config/config.yaml",
        help="Path to configuration file (default: config/config.yaml)",
        type=str,
    )
    arguments = parser.parse_args()

    with open(
        Path(Path.cwd(), arguments.conf_file_path),
        encoding="utf-8",
    ) as f:
        config_eval = yaml.safe_load(f)["evaluation"]

    spawner = VehicleSpawner(config_eval)
    spawner.run(config_eval)


if __name__ == '__main__':
    main()
