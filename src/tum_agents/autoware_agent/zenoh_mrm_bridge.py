#!/usr/bin/env python3
"""
zenoh_mrm_bridge.py

RUNS ON: Minerva (inside the Autoware container, Terminal 4)

PURPOSE:
    Listens for MRM command strings published by Vast.ai (Cosmos inference)
    over Zenoh and translates them into ROS2 MRM service calls on Autoware.

ZENOH KEY SUBSCRIBED:
    minerva/mrm/command   — expects one of: "emergency_stop", "comfortable_stop",
                            "pull_over", "cancel_all", "normal"

ZENOH KEY PUBLISHED:
    minerva/mrm/ack       — publishes acknowledgement after each service call

AUTOWARE SERVICES CALLED:
    /system/mrm/emergency_stop/operate
    /system/mrm/comfortable_stop/operate
    /system/mrm/pull_over_manager/operate


RUN (while Autoware is running):
    python3 zenoh_mrm_bridge.py
    python3 zenoh_mrm_bridge.py --minerva-endpoint tcp/0.0.0.0:7448  # separate port from camera
    python3 zenoh_mrm_bridge.py --vastai-endpoint tcp/<vastai_ip>:7447
"""

import argparse
import threading
import time

import rclpy
from rclpy.node import Node
from tier4_system_msgs.srv import OperateMrm

import zenoh

# MRM service paths (confirmed from ros2 service list)
MRM_SERVICES = {
    'emergency_stop':'/system/mrm/emergency_stop/operate',
    'comfortable_stop':'/system/mrm/comfortable_stop/operate',
    'pull_over':'/system/mrm/pull_over_manager/operate',
}

ZENOH_CMD_KEY = 'minerva/mrm/command'
ZENOH_ACK_KEY = 'minerva/mrm/ack'

VALID_COMMANDS = list(MRM_SERVICES.keys()) + ['cancel_all', 'normal']


# ROS2 MRM Service Caller 

class MrmServiceNode(Node):
    def __init__(self):
        super().__init__('zenoh_mrm_bridge_node')
        self._mrm_clients = {}
        for maneuver, path in MRM_SERVICES.items():
            self._mrm_clients[maneuver] = self.create_client(OperateMrm, path)
        self._active = None
        self.get_logger().info("MRM service clients created.")

    def call_mrm(self, maneuver: str, operate: bool) -> dict:
        # Call a single MRM service. Returns dict with success and message.
        client = self._mrm_clients[maneuver]
        if not client.wait_for_service(timeout_sec=3.0):
            msg = f"Service {MRM_SERVICES[maneuver]} unavailable"
            self.get_logger().error(msg)
            return {'success': False, 'message': msg}

        req = OperateMrm.Request()
        req.operate = operate
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            msg = f"Service call to {maneuver} timed out"
            self.get_logger().error(msg)
            return {'success': False, 'message': msg}

        result = future.result()
        action = "activated" if operate else "cancelled"
        self.get_logger().info(
            f"MRM {maneuver} {action}: "
            f"success={result.response.success} msg='{result.response.message}'"
        )
        return {
            'success': result.response.success,
            'message': result.response.message,
            'maneuver': maneuver,
            'operate': operate
        }

    def activate(self, maneuver: str) -> dict:
        return self.call_mrm(maneuver, operate=True)

    def cancel(self, maneuver: str) -> dict:
        return self.call_mrm(maneuver, operate=False)

    def cancel_all(self) -> list:
        results = []
        for m in MRM_SERVICES:
            results.append(self.call_mrm(m, operate=False))
        self._active = None
        return results


# Zenoh MRM Bridge 

class ZenohMrmBridge:
    def __init__(self, mrm_node: MrmServiceNode, zenoh_session):
        self._node = mrm_node
        self._session = zenoh_session
        self._ack_pub = zenoh_session.declare_publisher(ZENOH_ACK_KEY)
        self._lock = threading.Lock()

    def _handle_command(self, sample):
        """Called by Zenoh when a new MRM command arrives."""
        command = bytes(sample.payload).decode('utf-8').strip().lower()
        self._node.get_logger().info(f"[Zenoh] Received MRM command: '{command}'")

        if command not in VALID_COMMANDS:
            ack = f"ERROR: unknown command '{command}'. Valid: {VALID_COMMANDS}"
            self._node.get_logger().warn(ack)
            self._ack_pub.put(ack.encode())
            return

        with self._lock:
            if command == 'cancel_all' or command == 'normal':
                results = self._node.cancel_all()
                ack = f"cancelled_all ok={all(r['success'] for r in results)}"
            else:
                result = self._node.activate(command)
                ack = f"{command} ok={result['success']} msg={result['message']}"

        self._ack_pub.put(ack.encode())
        self._node.get_logger().info(f"[Zenoh] ACK published: {ack}")

    def start(self):
        self._sub = self._session.declare_subscriber(
            ZENOH_CMD_KEY,
            self._handle_command
        )
        self._node.get_logger().info(
            f"[Zenoh] Subscribed to MRM commands on key: {ZENOH_CMD_KEY}"
        )


# Main

def main():
    parser = argparse.ArgumentParser(
        description='Zenoh MRM bridge — receive Cosmos MRM decisions and call Autoware services'
    )
    parser.add_argument(
        '--minerva-endpoint', default='tcp/0.0.0.0:7448',
        help='Zenoh listen endpoint on Minerva (default: tcp/0.0.0.0:7448)'
    )
    parser.add_argument(
        '--vastai-endpoint', default=None,
        help='Optional: Vast.ai Zenoh endpoint to connect to, e.g. tcp/<vastai_ip>:7447'
    )
    args = parser.parse_args()

    # Zenoh session 
    zenoh_config = zenoh.Config()
    zenoh_config.insert_json5('listen/endpoints', f'["{args.minerva_endpoint}"]')
    if args.vastai_endpoint:
        zenoh_config.insert_json5('connect/endpoints', f'["{args.vastai_endpoint}"]')
        print(f"[Zenoh] Connecting to Vast.ai at {args.vastai_endpoint}")

    print(f"[Zenoh] Opening session on {args.minerva_endpoint}...")
    session = zenoh.open(zenoh_config)
    print(f"[Zenoh] Session open. Waiting for MRM commands on '{ZENOH_CMD_KEY}'...")

    # ROS2
    rclpy.init()
    mrm_node = MrmServiceNode()

    bridge = ZenohMrmBridge(mrm_node, session)
    bridge.start()

    # Spin ROS2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(mrm_node,), daemon=True)
    spin_thread.start()

    print("[INFO] MRM bridge running. Press Ctrl+C to stop.")
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down.")
    finally:
        mrm_node.destroy_node()
        rclpy.shutdown()
        session.close()
        print("[INFO] Zenoh session closed.")


if __name__ == '__main__':
    main()