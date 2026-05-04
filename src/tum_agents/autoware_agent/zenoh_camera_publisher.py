#!/usr/bin/env python3
"""
zenoh_camera_publisher.py

RUNS ON: Minerva (inside the bridge container, Terminal 2/3)

PURPOSE:
    Subscribes to the CARLA ego vehicle camera topic via ROS2,
    compresses each frame as JPEG, and publishes it to Zenoh so
    Vast.ai (Cosmos Reason 1) can subscribe and run inference.

ZENOH KEY:
    minerva/camera/front/image   — compressed JPEG bytes

RUN:
    python3 zenoh_camera_publisher.py
    python3 zenoh_camera_publisher.py --zenoh-endpoint tcp/0.0.0.0:7447  # listen on all interfaces
    python3 zenoh_camera_publisher.py --jpeg-quality 60                   # lower quality = less bandwidth
"""

import argparse
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import zenoh

# ── Configuration ──────────────────────────────────────────────────────────────

ROS2_CAMERA_TOPIC = '/carla/ego_vehicle/sensor/camera/image'
ZENOH_CAMERA_KEY  = 'minerva/camera/front/image'
ZENOH_STATUS_KEY  = 'minerva/camera/front/status'   # publishes FPS and frame count

# ── ROS2 Node ──────────────────────────────────────────────────────────────────

class CameraZenohPublisher(Node):
    def __init__(self, zenoh_session, jpeg_quality: int = 75):
        super().__init__('zenoh_camera_publisher_node')
        self._session = zenoh_session
        self._jpeg_quality = jpeg_quality
        self._publisher = zenoh_session.declare_publisher(ZENOH_CAMERA_KEY)
        self._frame_count = 0
        self._last_fps_time = time.monotonic()
        self._fps = 0.0

        self._sub = self.create_subscription(
            Image,
            ROS2_CAMERA_TOPIC,
            self._image_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Subscribed to {ROS2_CAMERA_TOPIC} → Zenoh key '{ZENOH_CAMERA_KEY}'"
        )

    def _image_callback(self, msg: Image):
        # Convert ROS2 Image to numpy array
        # CARLA publishes BGRA8 (4 channels)
        dtype = np.uint8
        channels = 4 if msg.encoding in ('bgra8', 'rgba8') else 3
        array = np.frombuffer(msg.raw_data if hasattr(msg, 'raw_data') else bytes(msg.data),
                              dtype=dtype)
        try:
            array = array.reshape((msg.height, msg.width, channels))
        except ValueError:
            # fallback: try to reshape based on data length
            channels = len(array) // (msg.height * msg.width)
            array = array.reshape((msg.height, msg.width, channels))

        # Convert to BGR for cv2 (drop alpha if present)
        if channels == 4:
            bgr = array[:, :, :3]          # drop alpha
            bgr = bgr[:, :, ::-1].copy()   # BGRA→BGR or RGBA→RGB→BGR
        else:
            bgr = array

        # JPEG compress
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if not ok:
            self.get_logger().warn("JPEG encoding failed, skipping frame.")
            return

        # Publish compressed bytes to Zenoh
        self._publisher.put(bytes(buf))
        self._frame_count += 1

        # FPS logging every 5 seconds
        now = time.monotonic()
        if now - self._last_fps_time >= 5.0:
            self._fps = self._frame_count / (now - self._last_fps_time)
            self.get_logger().info(
                f"Publishing at {self._fps:.1f} FPS | "
                f"frame size: {len(buf)/1024:.1f} KB | "
                f"total frames: {self._frame_count}"
            )
            # Publish status to Zenoh for monitoring
            status = f"fps={self._fps:.1f} frames={self._frame_count} quality={self._jpeg_quality}"
            self._session.put(ZENOH_STATUS_KEY, status.encode())
            self._frame_count = 0
            self._last_fps_time = now


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Bridge ROS2 camera topic to Zenoh for Vast.ai Cosmos inference'
    )
    parser.add_argument(
        '--zenoh-endpoint', default='tcp/0.0.0.0:7447',
        help='Zenoh listen endpoint (default: tcp/0.0.0.0:7447)'
    )
    parser.add_argument(
        '--vastai-endpoint', default=None,
        help='Optional: Vast.ai Zenoh endpoint to connect to, e.g. tcp/<vastai_ip>:7447'
    )
    parser.add_argument(
        '--jpeg-quality', type=int, default=75,
        help='JPEG compression quality 1-100 (default: 75). Lower = less bandwidth.'
    )
    parser.add_argument(
        '--ros-topic', default=ROS2_CAMERA_TOPIC,
        help=f'ROS2 camera topic (default: {ROS2_CAMERA_TOPIC})'
    )
    args = parser.parse_args()

    # ── Zenoh session ──────────────────────────────────────────────────────────
    zenoh_config = zenoh.Config()

    # Listen for incoming connections from Vast.ai
    zenoh_config.insert_json5(
        'listen/endpoints',
        f'["{args.zenoh_endpoint}"]'
    )

    # Optionally connect to Vast.ai as well (bidirectional)
    if args.vastai_endpoint:
        zenoh_config.insert_json5(
            'connect/endpoints',
            f'["{args.vastai_endpoint}"]'
        )
        print(f"[Zenoh] Will connect to Vast.ai at {args.vastai_endpoint}")

    print(f"[Zenoh] Opening session, listening on {args.zenoh_endpoint}...")
    session = zenoh.open(zenoh_config)
    print(f"[Zenoh] Session open. Publishing camera to key: {ZENOH_CAMERA_KEY}")

    # ── ROS2 ──────────────────────────────────────────────────────────────────
    rclpy.init()
    node = CameraZenohPublisher(session, jpeg_quality=args.jpeg_quality)

    print(f"[ROS2] Subscribed to {args.ros_topic}")
    print("[INFO] Waiting for camera frames... Press Ctrl+C to stop.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        session.close()
        print("[INFO] Zenoh session closed.")


if __name__ == '__main__':
    main()