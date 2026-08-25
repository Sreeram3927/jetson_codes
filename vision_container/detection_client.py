import socket
import json
import time
import threading


class DetectionSender:
    """
    Sends detection JSON lines to the ROS2 detection_relay node over a
    plain TCP socket. This process never imports rclpy — it stays on
    Python 3.8 (required by ultralytics/torch) while the ROS2 side stays
    on whatever Python version its rclpy build requires.

    Reconnects lazily: if the ROS2 container isn't up yet, or restarts,
    frames are dropped rather than blocking the vision loop, and the next
    send() call retries the connection.
    """

    def __init__(self, host=None, port=None, retry_interval=2.0):
        import os
        self.host = host or os.environ.get("RELAY_HOST", "localhost")
        self.port = int(port or os.environ.get("RELAY_PORT", 9999))
        self.retry_interval = retry_interval
        self.sock = None
        self.lock = threading.Lock()
        self._last_attempt = 0.0
        self._connect()

    def _connect(self):
        self._last_attempt = time.time()
        try:
            s = socket.create_connection((self.host, self.port), timeout=2.0)
            with self.lock:
                self.sock = s
            print(f"[DetectionSender] Connected to relay at {self.host}:{self.port}")
        except OSError as e:
            print(f"[DetectionSender] Relay not reachable yet ({e}); will retry")
            with self.lock:
                self.sock = None

    def send(self, targets):
        """targets: list of dicts, e.g. [{'x':.., 'y':.., 'z':.., 'conf':..}, ...]"""
        with self.lock:
            sock = self.sock

        if sock is None:
            # Don't hammer reconnect attempts every frame
            if time.time() - self._last_attempt >= self.retry_interval:
                self._connect()
                with self.lock:
                    sock = self.sock
            if sock is None:
                return  # still down; drop this frame's detections

        payload = json.dumps({"timestamp": time.time(), "targets": targets}) + "\n"
        try:
            sock.sendall(payload.encode("utf-8"))
        except OSError as e:
            print(f"[DetectionSender] Send failed ({e}); will reconnect on next send")
            with self.lock:
                self.sock = None

    def close(self):
        with self.lock:
            if self.sock:
                self.sock.close()
                self.sock = None
