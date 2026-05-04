#!/usr/bin/env python3
"""
zenoh_cosmos_client.py

RUNS ON: Vast.ai GPU instance

PURPOSE:
    - Subscribes to camera frames from Minerva over Zenoh
    - Buffers frames into short clips
    - Runs Cosmos Reason 1 inference on each clip
    - Publishes MRM decision back to Minerva over Zenoh

SETUP ON VAST.AI:
    pip install eclipse-zenoh opencv-python openai requests numpy

    # Set your Cosmos API endpoint and key:
    export COSMOS_API_URL="https://your-cosmos-endpoint/v1"
    export COSMOS_API_KEY="your-api-key"

RUN:
    python3 zenoh_cosmos_client.py --minerva-ip <minerva_ip>
    python3 zenoh_cosmos_client.py --minerva-ip 10.20.90.208 --clip-frames 10 --inference-interval 5

ZENOH KEYS:
    SUBSCRIBES: minerva/camera/front/image    — JPEG compressed frames from Minerva
    PUBLISHES:  minerva/mrm/command           — MRM decision string
                minerva/mrm/cosmos_reasoning  — full Cosmos reasoning text (for logging)
"""

import argparse
import base64
import os
import time
import threading
import queue
from collections import deque
import numpy as np
import cv2
import zenoh
import requests
import json

# Configuration

ZENOH_CAMERA_KEY   = 'minerva/camera/front/image'
ZENOH_MRM_CMD_KEY  = 'minerva/mrm/command'
ZENOH_REASON_KEY   = 'minerva/mrm/cosmos_reasoning'
ZENOH_ACK_KEY      = 'minerva/mrm/ack'

# MRM output keywords Cosmos must choose from
MRM_LABELS = {
    'Emergency_Stop':   'emergency_stop',
    'Comfortable_Stop': 'comfortable_stop',
    'Pull_Over':        'pull_over',
    'Normal':           'normal',
}

# These 2 prompts might be clubbed.

COSMOS_SYSTEM_PROMPT = """The Prompt..."""

COSMOS_USER_PROMPT = """Here are {n_frames} sequential frames from the ego vehicle's front camera.
Classify the scenario into exactly one of: Emergency_Stop, Comfortable_Stop, Pull_Over, Normal.
Respond with ONLY the label."""


# Cosmos Inference

class CosmosInference:
    def __init__(self, api_url: str, api_key: str, model: str = "cosmos-reason1"):
        self._api_url = api_url.rstrip('/')
        self._headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }
        self._model = model

    def classify(self, frames: list) -> tuple:
        
        # Run Cosmos inference on a list of BGR numpy frames.
        # Returns (mrm_label, raw_reasoning_text).
        
        # Encode frames as base64 JPEG
        image_messages = []
        for frame in frames:
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                continue
            b64 = base64.b64encode(bytes(buf)).decode('utf-8')
            image_messages.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{b64}"}
            })

        if not image_messages:
            return 'normal', 'No frames to process'

        payload = {
            "model": self._model,
            "messages": [
                {"role": "system", "content": COSMOS_SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": image_messages + [
                        {"type": "text",
                         "text": COSMOS_USER_PROMPT.format(n_frames=len(frames))}
                    ]
                }
            ],
            "max_tokens": 50,
            "temperature": 0.0
        }

        try:
            resp = requests.post(
                f"{self._api_url}/chat/completions",
                headers=self._headers,
                json=payload,
                timeout=30.0
            )
            resp.raise_for_status()
            raw = resp.json()['choices'][0]['message']['content'].strip()
        except Exception as e:
            print(f"[Cosmos] Inference error: {e}")
            return 'normal', f'error: {e}'

        # Parse label
        label = 'normal'
        for cosmos_label, mrm_label in MRM_LABELS.items():
            if cosmos_label.lower() in raw.lower():
                label = mrm_label
                break

        return label, raw


# Zenoh Cosmos Client

class ZenohCosmosClient:
    def __init__(self, session, cosmos: CosmosInference,
                 clip_frames: int = 8, inference_interval: float = 3.0):
        self._session = session
        self._cosmos = cosmos
        self._clip_frames = clip_frames
        self._inference_interval = inference_interval

        self._frame_buffer = deque(maxlen=clip_frames)
        self._inference_queue = queue.Queue(maxsize=1)
        self._last_inference_time = 0.0
        self._last_decision = 'normal'
        self._frame_count = 0

        # Publishers
        self._mrm_pub = session.declare_publisher(ZENOH_MRM_CMD_KEY)
        self._reason_pub = session.declare_publisher(ZENOH_REASON_KEY)

    def _frame_callback(self, sample):
        """Called by Zenoh when a new compressed frame arrives from Minerva."""
        jpg_bytes = bytes(sample.payload)
        arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        self._frame_buffer.append(frame)
        self._frame_count += 1

        # Trigger inference if enough time has passed and buffer is full
        now = time.monotonic()
        if (len(self._frame_buffer) >= self._clip_frames and
                now - self._last_inference_time >= self._inference_interval):
            try:
                self._inference_queue.put_nowait(list(self._frame_buffer))
                self._last_inference_time = now
            except queue.Full:
                pass  # inference still running from previous clip

    def _inference_worker(self):
        """Background thread: runs Cosmos inference and publishes MRM command."""
        print("[Cosmos] Inference worker started.")
        while True:
            try:
                frames = self._inference_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            print(f"[Cosmos] Running inference on {len(frames)} frames...")
            t0 = time.monotonic()
            decision, reasoning = self._cosmos.classify(frames)
            latency = time.monotonic() - t0

            print(f"[Cosmos] Decision: {decision} | Latency: {latency:.2f}s")
            print(f"[Cosmos] Reasoning: {reasoning}")

            # Publish MRM command to Minerva
            self._mrm_pub.put(decision.encode())

            # Publish full reasoning for logging
            log = json.dumps({
                'decision': decision,
                'reasoning': reasoning,
                'latency_sec': round(latency, 3),
                'frames': len(frames),
                'timestamp': time.time()
            })
            self._reason_pub.put(log.encode())

            self._last_decision = decision
            self._inference_queue.task_done()

    def _ack_callback(self, sample):
        """Log acknowledgements from Minerva's MRM bridge."""
        ack = bytes(sample.payload).decode('utf-8')
        print(f"[Minerva ACK] {ack}")

    def start(self, session):
        # Subscribe to camera frames from Minerva
        self._camera_sub = session.declare_subscriber(
            ZENOH_CAMERA_KEY, self._frame_callback
        )
        # Subscribe to ACKs from Minerva
        self._ack_sub = session.declare_subscriber(
            ZENOH_ACK_KEY, self._ack_callback
        )

        # Start inference in background thread
        self._worker_thread = threading.Thread(
            target=self._inference_worker, daemon=True
        )
        self._worker_thread.start()

        print(f"[Zenoh] Subscribed to camera: {ZENOH_CAMERA_KEY}")
        print(f"[Zenoh] Publishing MRM commands to: {ZENOH_MRM_CMD_KEY}")


# Main
def main():
    parser = argparse.ArgumentParser(
        description='Zenoh Cosmos client — receive camera frames, run inference, send MRM commands'
    )
    parser.add_argument(
        '--minerva-ip', required=True,
        help='IP address of Minerva, e.g. 10.20.90.208'
    )
    parser.add_argument(
        '--minerva-camera-port', default=7447, type=int,
        help='Zenoh port for camera stream on Minerva (default: 7447)'
    )
    parser.add_argument(
        '--minerva-mrm-port', default=7448, type=int,
        help='Zenoh port for MRM bridge on Minerva (default: 7448)'
    )
    parser.add_argument(
        '--clip-frames', default=8, type=int,
        help='Number of frames per inference clip (default: 8)'
    )
    parser.add_argument(
        '--inference-interval', default=3.0, type=float,
        help='Minimum seconds between inference calls (default: 3.0)'
    )
    parser.add_argument(
        '--cosmos-url', default=os.environ.get('COSMOS_API_URL', ''),
        help='Cosmos API URL (or set COSMOS_API_URL env var)'
    )
    parser.add_argument(
        '--cosmos-key', default=os.environ.get('COSMOS_API_KEY', ''),
        help='Cosmos API key (or set COSMOS_API_KEY env var)'
    )
    parser.add_argument(
        '--cosmos-model', default='cosmos-reason1',
        help='Cosmos model name (default: cosmos-reason1)'
    )
    args = parser.parse_args()

    if not args.cosmos_url or not args.cosmos_key:
        print("ERROR: Set --cosmos-url and --cosmos-key or COSMOS_API_URL/COSMOS_API_KEY env vars.")
        return

    # Zenoh session
    # Connect to BOTH Minerva endpoints: camera publisher and MRM bridge
    camera_ep = f"tcp/{args.minerva_ip}:{args.minerva_camera_port}"
    mrm_ep    = f"tcp/{args.minerva_ip}:{args.minerva_mrm_port}"

    zenoh_config = zenoh.Config()
    zenoh_config.insert_json5(
        'connect/endpoints',
        f'["{camera_ep}", "{mrm_ep}"]'
    )

    print(f"[Zenoh] Connecting to Minerva camera at {camera_ep}")
    print(f"[Zenoh] Connecting to Minerva MRM bridge at {mrm_ep}")
    session = zenoh.open(zenoh_config)
    print("[Zenoh] Session open.")

    # Cosmos
    cosmos = CosmosInference(args.cosmos_url, args.cosmos_key, args.cosmos_model)

    # Client
    client = ZenohCosmosClient(
        session, cosmos,
        clip_frames=args.clip_frames,
        inference_interval=args.inference_interval
    )
    client.start(session)

    print(f"\n[INFO] Running. Inference every {args.inference_interval}s on {args.clip_frames}-frame clips.")
    print("[INFO] Press Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down.")
    finally:
        session.close()
        print("[INFO] Zenoh session closed.")


if __name__ == '__main__':
    main()