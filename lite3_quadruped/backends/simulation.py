"""MuJoCo/ROS 2 backend with no physical-robot network operations."""

from __future__ import annotations

import logging
import subprocess
import time

from .base import Backend, RequiredTopic
from ..config import RuntimeConfig


log = logging.getLogger("lite3.simulation")

POSTURE_SERVICES = {
    "stand": "/lite3_sim/stand",
    "sit": "/lite3_sim/lie_down",
    "lie_down": "/lite3_sim/lie_down",
    "zero": "/lite3_sim/reset",
    "reset": "/lite3_sim/reset",
    "init": "/lite3_sim/reset",
    "emergency_stop": "/lite3_sim/emergency_stop",
}


class SimulationBackend(Backend):
    name = "simulation"

    def prepare(self) -> None:
        log.info("simulation backend uses external Lite3 MuJoCo Bridge")

    def required_topics(self) -> tuple[RequiredTopic, ...]:
        return (
            RequiredTopic("/clock", "rosgraph_msgs/msg/Clock"),
            RequiredTopic(self.config.odom_topic, "nav_msgs/msg/Odometry"),
            RequiredTopic("/joint_states", "sensor_msgs/msg/JointState"),
        )

    def wait_for_topic(self, _capability, topic: RequiredTopic) -> bool:
        """Use a ROS graph query to avoid Fast DDS short-lived subscription races."""
        deadline = time.monotonic() + self.config.sentinel_timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.1, deadline - time.monotonic())
            try:
                result = subprocess.run(
                    ["ros2", "topic", "type", topic.name],
                    capture_output=True,
                    text=True,
                    timeout=min(5.0, remaining),
                    check=False,
                )
            except subprocess.TimeoutExpired:
                continue
            if result.returncode == 0 and result.stdout.strip() == topic.msg_type:
                log.info("discovered topic: %s [%s]", topic.name, topic.msg_type)
                return True
            time.sleep(min(0.2, remaining))
        return False

    def verify_activity(self) -> None:
        for topic in self.required_topics():
            try:
                result = subprocess.run(
                    ["ros2", "topic", "echo", "--once", topic.name,
                     topic.msg_type],
                    capture_output=True,
                    text=True,
                    timeout=self.config.sentinel_timeout_s,
                    check=False,
                )
            except subprocess.TimeoutExpired as error:
                raise RuntimeError(
                    f"topic {topic.name} exists but did not publish in time"
                ) from error
            if result.returncode != 0:
                detail = (result.stderr or result.stdout).strip()
                raise RuntimeError(
                    f"topic {topic.name} exists but is not updating: {detail}")
            log.info("active topic verified: %s", topic.name)

    def activate(self) -> None:
        log.info("simulation backend active; no hardware UDP was opened")

    def set_posture(self, posture: str) -> tuple[bool, str]:
        service = POSTURE_SERVICES.get(posture)
        if service is None:
            return False, (
                f"unknown simulation posture '{posture}'; "
                f"valid={sorted(POSTURE_SERVICES)}")
        try:
            result = subprocess.run(
                ["ros2", "service", "call", service,
                 "std_srvs/srv/Trigger", "{}"],
                capture_output=True,
                text=True,
                timeout=10.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return False, f"simulation service timed out: {service}"
        output = (result.stdout + result.stderr).strip()
        success = result.returncode == 0 and "success=True" in output
        if not success:
            return False, f"simulation service failed: {service}: {output}"
        return True, f"simulation posture '{posture}' accepted via {service}"

    def shutdown(self) -> None:
        # Best effort only: the bridge may already have stopped.
        try:
            subprocess.run(
                ["ros2", "service", "call", "/lite3_sim/emergency_stop",
                 "std_srvs/srv/Trigger", "{}"],
                capture_output=True,
                timeout=2.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass
        log.info("simulation backend stopped")
