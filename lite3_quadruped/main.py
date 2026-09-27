#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Lite3 chassis primitive with hardware and MuJoCo simulation backends."""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from pathlib import Path

from robonix_api import Primitive, Ok, Err, Deferred

from .backends import Backend, create_backend
from .config import ConfigError, parse_config

logging.basicConfig(
    level=os.environ.get("LITE3_LOG_LEVEL", "INFO"),
    format="[lite3] %(message)s",
)
log = logging.getLogger("lite3")

lite3_quadruped = Primitive(id="lite3_quadruped", namespace="robonix/primitive/chassis")

cmd_vel_pub = None
_odom_yaw: float | None = None
_odom_stamp = 0.0
_odom_cv = threading.Condition()
_motion_lock = threading.Lock()

_pkg_root = Path(
    os.environ.get("RBNX_PACKAGE_ROOT", Path(__file__).resolve().parent.parent)
).resolve()
_backend: Backend | None = None

import chassis_pb2  # noqa: E402
import std_msgs_pb2  # noqa: E402
from quadruped_mcp import SetPosture_Request, SetPosture_Response  # noqa: E402


@lite3_quadruped.grpc(
    "robonix/primitive/chassis/move",
    description=(
        "Move or rotate the Lite3 base. Use forward_m for signed forward/backward "
        "distance, rotate_deg for signed in-place yaw rotation (positive is "
        "counter-clockwise/left, negative is clockwise/right), or angular_z with "
        "duration_sec for raw yaw velocity. The robot must already be standing."
    ),
)
def move(req: "chassis_pb2.ExecuteMoveCommand_Request") -> "chassis_pb2.ExecuteMoveCommand_Response":
    global cmd_vel_pub
    if cmd_vel_pub is None:
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(data=json.dumps({"error": "ROS2 not initialized"})),
        )

    msg = req.command
    speed_mps = float(os.environ.get("LITE3_QUADRUPED_SPEED_MPS", "0.2"))
    ang_speed_rps = float(os.environ.get("LITE3_QUADRUPED_ANG_SPEED_RPS", "0.6"))
    default_dur = float(os.environ.get("LITE3_QUADRUPED_CMD_DURATION_SEC", "1.0"))

    from geometry_msgs.msg import Twist  # type: ignore

    with _motion_lock:
        tw = Twist()
        actual_rotate_deg = 0.0
        if float(getattr(msg, "forward_m", 0.0)) != 0.0:
            forward_m = float(msg.forward_m)
            sign = 1.0 if forward_m > 0 else -1.0
            tw.linear.x = sign * speed_mps
            duration = abs(forward_m) / speed_mps
            mode = "forward_m"
            _publish_for_duration(tw, duration)
        elif float(getattr(msg, "rotate_deg", 0.0)) != 0.0:
            rotate_deg = float(msg.rotate_deg)
            ok, actual_rotate_deg, duration, error = _rotate_closed_loop(
                rotate_deg, ang_speed_rps,
            )
            if not ok:
                return chassis_pb2.ExecuteMoveCommand_Response(
                    status=std_msgs_pb2.String(data=json.dumps({
                        "status": "error",
                        "mode": "rotate_deg",
                        "error": error,
                        "requested_rotate_deg": rotate_deg,
                        "actual_rotate_deg": actual_rotate_deg,
                    })),
                )
            mode = "rotate_deg"
        else:
            tw.linear.x = float(getattr(msg, "linear_x", 0.0))
            tw.linear.y = float(getattr(msg, "linear_y", 0.0))
            tw.linear.z = float(getattr(msg, "linear_z", 0.0))
            tw.angular.x = float(getattr(msg, "angular_x", 0.0))
            tw.angular.y = float(getattr(msg, "angular_y", 0.0))
            tw.angular.z = float(getattr(msg, "angular_z", 0.0))
            duration = float(getattr(msg, "duration_sec", 0.0))
            if duration <= 0.0:
                duration = default_dur
            mode = "velocity"
            _publish_for_duration(tw, duration)

    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps({
            "status": "done",
            "mode": mode,
            "duration_sec": duration,
            "linear_x": tw.linear.x,
            "angular_z": tw.angular.z,
            "actual_rotate_deg": actual_rotate_deg,
        })),
    )


def _publish_for_duration(twist, duration: float) -> None:
    """Publish a velocity at 10 Hz and always finish with a stop command."""
    from geometry_msgs.msg import Twist  # type: ignore

    try:
        deadline = time.monotonic() + max(0.1, duration)
        while time.monotonic() < deadline:
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
    finally:
        cmd_vel_pub.publish(Twist())


def _wrap_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def _on_odom(msg) -> None:
    """Track the latest odometry yaw for closed-loop angle commands."""
    global _odom_yaw, _odom_stamp
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    with _odom_cv:
        _odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        _odom_stamp = time.monotonic()
        _odom_cv.notify_all()


def _rotate_closed_loop(target_deg: float, max_speed: float):
    """Rotate using accumulated odometry yaw, handling wrap at +/-pi."""
    from geometry_msgs.msg import Twist  # type: ignore

    target = math.radians(target_deg)
    tolerance = math.radians(float(os.environ.get(
        "LITE3_ROTATE_TOLERANCE_DEG", "3.0")))
    timeout = max(5.0, abs(target) / max(0.1, max_speed) * 2.5 + 2.0)
    started = time.monotonic()
    with _odom_cv:
        if _odom_yaw is None or started - _odom_stamp > 1.0:
            _odom_cv.wait(timeout=1.0)
        if _odom_yaw is None or time.monotonic() - _odom_stamp > 1.0:
            return False, 0.0, 0.0, "fresh odometry unavailable"
        previous = _odom_yaw

    accumulated = 0.0
    command = Twist()
    try:
        while time.monotonic() - started < timeout:
            with _odom_cv:
                _odom_cv.wait(timeout=0.1)
                if _odom_yaw is None or time.monotonic() - _odom_stamp > 1.0:
                    return False, math.degrees(accumulated), time.monotonic() - started, "odometry became stale"
                current = _odom_yaw
            accumulated += _wrap_angle(current - previous)
            previous = current
            remaining = target - accumulated
            if abs(remaining) <= tolerance:
                return True, math.degrees(accumulated), time.monotonic() - started, ""
            speed = min(max_speed, max(0.15, abs(remaining) * 1.2))
            command.angular.z = math.copysign(speed, remaining)
            cmd_vel_pub.publish(command)
        return False, math.degrees(accumulated), time.monotonic() - started, "rotation timeout"
    finally:
        cmd_vel_pub.publish(Twist())


@lite3_quadruped.mcp("robonix/primitive/chassis/posture")
def set_posture(req: SetPosture_Request) -> SetPosture_Response:
    """Route a posture request to the selected backend."""
    if _backend is None:
        return SetPosture_Response(
            success=False, message="Lite3 backend is not initialized")
    try:
        success, message = _backend.set_posture(req.posture_name)
    except (OSError, RuntimeError) as error:
        success, message = False, f"posture request failed: {error}"
    return SetPosture_Response(success=success, message=message)


@lite3_quadruped.on_init
def init(cfg):
    """Select a backend, register common capabilities, and check sentinels."""
    global cmd_vel_pub, _backend

    try:
        runtime = parse_config(cfg)
        _backend = create_backend(runtime, _pkg_root)
        log.info("selected backend=%s", _backend.name)
        _backend.prepare()
    except (ConfigError, OSError, ValueError, RuntimeError) as error:
        if _backend is not None:
            _backend.shutdown()
        _backend = None
        return Err(f"Lite3 backend initialization failed: {error}")

    try:
        from geometry_msgs.msg import Twist  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore

        cmd_vel_pub = lite3_quadruped.create_publisher(
            "robonix/primitive/chassis/twist_in",
            topic=runtime.twist_in_topic,
            msg_type=Twist,
            qos="reliable",
            declare=False,
        )
        lite3_quadruped.declare_ros2_topic(
            "robonix/primitive/chassis/twist_in",
            runtime.twist_in_topic,
            qos="reliable",
        )

        lite3_quadruped.create_subscription(
            "robonix/primitive/chassis/odom",
            topic=runtime.odom_topic,
            msg_type=Odometry,
            callback=_on_odom,
            qos="best_effort",
            declare=False,
        )
        lite3_quadruped.declare_ros2_topic(
            "robonix/primitive/chassis/odom",
            runtime.odom_topic,
            qos="reliable",
        )

        for topic in _backend.required_topics():
            if not _backend.wait_for_topic(lite3_quadruped, topic):
                _backend.shutdown()
                _backend = None
                return Deferred(
                    f"{runtime.backend} topic {topic.name} is unavailable")
    except Exception as error:
        _backend.shutdown()
        _backend = None
        cmd_vel_pub = None
        return Err(f"failed to register Lite3 ROS interfaces: {error}")

    try:
        _backend.verify_activity()
        _backend.activate()
    except (OSError, RuntimeError) as error:
        _backend.shutdown()
        _backend = None
        return Deferred(f"{runtime.backend} backend is not ready: {error}")

    return Ok()


@lite3_quadruped.on_shutdown
def shutdown():
    """Stop motion and shut down only the selected backend."""
    global cmd_vel_pub, _backend
    if cmd_vel_pub is not None:
        try:
            from geometry_msgs.msg import Twist  # type: ignore
            cmd_vel_pub.publish(Twist())
        except Exception as error:
            log.warning("failed to publish shutdown stop command: %s", error)
    if _backend is not None:
        try:
            _backend.shutdown()
        except Exception as error:
            log.warning("backend shutdown failed: %s", error)
    _backend = None
    cmd_vel_pub = None
    return Ok()


if __name__ == "__main__":
    lite3_quadruped.run()
