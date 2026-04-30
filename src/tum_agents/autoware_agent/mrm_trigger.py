#!/usr/bin/env python3
"""
mrm_trigger.py

LOCATION:
    /data/vedant/Ubuntu/Swaraj/autoware_carla_leaderboard/src/tum_agents/autoware_agent/mrm_trigger.py

PURPOSE:
    Trigger MRM maneuvers in Autoware from the VLM (Cosmos Reason) observer layer.
    Calls the MRM operator service endpoints directly.

SERVICES CALLED:
    /system/mrm/emergency_stop/operate      — hard brake, imminent threat
    /system/mrm/comfortable_stop/operate    — controlled deceleration, hazard present
    /system/mrm/pull_over_manager/operate   — maneuver to roadside, sufficient time/space

USAGE (standalone test):
    python3 mrm_trigger.py --maneuver emergency_stop
    python3 mrm_trigger.py --maneuver comfortable_stop
    python3 mrm_trigger.py --maneuver pull_over
    python3 mrm_trigger.py --maneuver cancel_all

USAGE (from reasoning layer):
    from mrm_trigger import MrmTrigger
    trigger = MrmTrigger()
    trigger.activate('emergency_stop')   # or 'comfortable_stop' or 'pull_over'
    trigger.cancel_all()
    trigger.destroy()
"""

import rclpy
from rclpy.node import Node
from tier4_system_msgs.srv import OperateMrm
import argparse
import sys
import time


# Resolved service paths confirmed from `ros2 service list`
MRM_SERVICES = {
    'emergency_stop':   '/system/mrm/emergency_stop/operate',
    'comfortable_stop': '/system/mrm/comfortable_stop/operate',
    'pull_over':        '/system/mrm/pull_over_manager/operate',
}

# Priority order — higher index = higher priority
MRM_PRIORITY = ['pull_over', 'comfortable_stop', 'emergency_stop']


class MrmTrigger(Node):
    
    # ROS2 node that calls MRM operator services to trigger or cancel maneuvers.


    def __init__(self, node_name: str = 'mrm_trigger_node'):
        super().__init__(node_name)
        self._mrm_clients = {}
        for maneuver, service_path in MRM_SERVICES.items():
            self._mrm_clients[maneuver] = self.create_client(OperateMrm, service_path)
        self._active_maneuver = None
        self.get_logger().info("MrmTrigger initialized.")

    def _wait_for_service(self, maneuver: str, timeout_sec: float = 3.0) -> bool:
        client = self._mrm_clients[maneuver]
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"Service {MRM_SERVICES[maneuver]} not available after {timeout_sec}s."
            )
            return False
        return True

    def _call(self, maneuver: str, operate: bool) -> bool:
        """Internal: call the service for a given maneuver."""
        if maneuver not in self._mrm_clients:
            self.get_logger().error(f"Unknown maneuver: {maneuver}")
            return False

        if not self._wait_for_service(maneuver):
            return False

        req = OperateMrm.Request()
        req.operate = operate

        client = self._mrm_clients[maneuver]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error(
                f"Service call to {MRM_SERVICES[maneuver]} timed out or failed."
            )
            return False

        result = future.result()
        action = "activated" if operate else "cancelled"
        self.get_logger().info(
            f"MRM {maneuver} {action} — "
            f"success={result.response.success} "
            f"code={result.response.code} "
            f"message='{result.response.message}'"
        )
        return result.response.success

    def activate(self, maneuver: str) -> bool:
        # Activate a maneuver. If a lower-priority maneuver is currently active,
        # cancel it first before activating the new one.
        # Priority (highest to lowest): emergency_stop > comfortable_stop > pull_over

        if maneuver not in MRM_SERVICES:
            self.get_logger().error(
                f"Invalid maneuver '{maneuver}'. "
                f"Choose from: {list(MRM_SERVICES.keys())}"
            )
            return False

        # Cancel existing lower-priority maneuver if active
        if (self._active_maneuver is not None and
                self._active_maneuver != maneuver and
                MRM_PRIORITY.index(self._active_maneuver) <
                MRM_PRIORITY.index(maneuver)):
            self.get_logger().info(
                f"Cancelling lower-priority {self._active_maneuver} "
                f"before activating {maneuver}."
            )
            self._call(self._active_maneuver, operate=False)
            self._active_maneuver = None

        success = self._call(maneuver, operate=True)
        if success:
            self._active_maneuver = maneuver
        return success

    def cancel(self, maneuver: str) -> bool:
        # Cancel a specific maneuver
        success = self._call(maneuver, operate=False)
        if success and self._active_maneuver == maneuver:
            self._active_maneuver = None
        return success

    def cancel_all(self) -> None:
        # Cancel all three maneuvers — use when returning to normal driving
        self.get_logger().info("Cancelling all MRM maneuvers.")
        for maneuver in MRM_SERVICES:
            self._call(maneuver, operate=False)
        self._active_maneuver = None

    def destroy(self):
        """Clean up node."""
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description='MRM trigger — activate or cancel MRM maneuvers in Autoware'
    )
    parser.add_argument(
        '--maneuver', required=True,
        choices=['emergency_stop', 'comfortable_stop', 'pull_over', 'cancel_all'],
        help=(
            'emergency_stop:   hard brake, imminent threat\n'
            'comfortable_stop: controlled decel, hazard present\n'
            'pull_over:        maneuver to roadside\n'
            'cancel_all:       cancel all active maneuvers'
        )
    )
    parser.add_argument(
        '--cancel', action='store_true',
        help='Cancel the specified maneuver instead of activating it'
    )
    args = parser.parse_args()

    rclpy.init()
    trigger = MrmTrigger()

    if args.maneuver == 'cancel_all':
        trigger.cancel_all()
    elif args.cancel:
        trigger.cancel(args.maneuver)
    else:
        trigger.activate(args.maneuver)

    trigger.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()