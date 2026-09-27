"""Physical Lite3 motion-host backend."""

from __future__ import annotations

import logging
import os
import signal
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

from .base import Backend, RequiredTopic
from ..config import RuntimeConfig


log = logging.getLogger("lite3.hardware")

COMMANDS = {
    "stand": (0x21010202, 0, 0),
    "sit": (0x21010202, 0, 0),
    "zero": (0x21010C05, 0, 0),
    "init": (0x21010C03, 0, 0),
    "remote": (0x21010C02, 0, 1),
}
HEARTBEAT = (0x21040001, 0, 0)


class HardwareBackend(Backend):
    name = "hardware"

    def __init__(self, config: RuntimeConfig, package_root: Path):
        super().__init__(config)
        raw = config.raw
        self.package_root = package_root
        self.target_ip = str(raw.get("target_ip", "192.168.1.120"))
        self.target_port = int(raw.get("target_port", 43893))
        self.local_port = int(raw.get("local_port", 43897))
        self.app_port = int(raw.get("app_port", 43899))
        self.publish_odom_tf = bool(raw.get("publish_odom_tf", True))
        self.transfer_process: subprocess.Popen | None = None
        self.transfer_log = None
        self.heartbeat_stop = threading.Event()
        self.heartbeat_thread: threading.Thread | None = None

    def prepare(self) -> None:
        env = dict(os.environ)
        env.update({
            "TRANSFER_TARGET_IP": self.target_ip,
            "TRANSFER_TARGET_PORT": str(self.target_port),
            "TRANSFER_LOCAL_PORT": str(self.local_port),
            "TRANSFER_APP_PORT": str(self.app_port),
            "TRANSFER_ODOM_TOPIC": self.config.odom_topic,
            "TRANSFER_PUBLISH_ODOM_TF": (
                "true" if self.publish_odom_tf else "false"),
        })
        log_path = self.package_root / "rbnx-build" / "data" / "transfer.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        env["TRANSFER_APP_LOG_PATH"] = str(log_path.with_name("nx2app.log"))
        self.transfer_log = open(log_path, "ab", buffering=0)
        log.info(
            "spawning transfer target=%s:%d odom=%s log=%s",
            self.target_ip, self.target_port, self.config.odom_topic, log_path,
        )
        self.transfer_process = subprocess.Popen(
            ["ros2", "launch", "transfer", "transfer_launch.py"],
            env=env,
            stdout=self.transfer_log,
            stderr=self.transfer_log,
            start_new_session=True,
        )

    def required_topics(self) -> tuple[RequiredTopic, ...]:
        return (RequiredTopic(self.config.odom_topic, "nav_msgs/msg/Odometry"),)

    def activate(self) -> None:
        self.heartbeat_stop.clear()
        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        self._send_named("init")
        time.sleep(2.0)

    def set_posture(self, posture: str) -> tuple[bool, str]:
        if posture not in COMMANDS:
            return False, f"unknown hardware posture '{posture}'"
        self._send_named(posture)
        time.sleep(1.0)
        return True, f"hardware posture '{posture}' dispatched"

    def shutdown(self) -> None:
        self.heartbeat_stop.set()
        if self.heartbeat_thread is not None:
            self.heartbeat_thread.join(timeout=1.0)
            self.heartbeat_thread = None
        process = self.transfer_process
        if process is not None and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5.0)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.transfer_process = None
        if self.transfer_log is not None:
            self.transfer_log.close()
            self.transfer_log = None
        log.info("hardware backend stopped")

    def _send_udp(self, command: tuple[int, int, int]) -> None:
        payload = struct.pack("<III", *command)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(payload, (self.target_ip, self.target_port))

    def _send_named(self, name: str) -> None:
        self._send_udp(COMMANDS[name])
        log.info("command '%s' sent to %s:%d", name, self.target_ip,
                 self.target_port)

    def _heartbeat_loop(self) -> None:
        # Preserve the physical driver behavior: send once immediately, then
        # continue at 200 ms intervals until shutdown.
        while not self.heartbeat_stop.is_set():
            try:
                self._send_udp(HEARTBEAT)
            except OSError as error:
                log.warning("heartbeat error: %s", error)
            if self.heartbeat_stop.wait(0.2):
                break
