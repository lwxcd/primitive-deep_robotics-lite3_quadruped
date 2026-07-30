#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""lite3_quadruped — Lite3 quadruped chassis primitive.

Owns ``robonix/primitive/chassis/*``.

Lifecycle:
    on_init     — spawn transfer ROS2 nodes (jetson2motion, jetson2app,
                  sensor_checker) → create cmd_vel publisher → wait for
                  /leg_odom2 → declare twist_in/odom contracts.
    on_shutdown — kill transfer subprocess.

Config (from manifest's ``config:`` block, delivered via Driver(CMD_INIT)):
    odom_topic       default "/leg_odom2"        (env LITE3_ODOM_TOPIC)
    twist_in_topic   default "/cmd_vel"           (env LITE3_CMD_VEL_TOPIC)
    target_ip        default "192.168.1.120"      motion host UDP address
    target_port      default 43893
    local_port       default 43897
    app_port         default 43899
    publish_odom_tf  default true (publish chassis odom -> base_link TF)
"""
from __future__ import annotations

import json
import logging
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import socket
import struct

from robonix_api import Primitive, Ok, Err, Deferred

ROBOT_IP = "192.168.1.120"
ROBOT_PORT = 43893

cmd_dict = {
    "stand": [0x21010202, 0, 0],
    "sit": [0x21010202, 0, 0],
    "zero": [0x21010C05, 0, 0],
    "init": [0x21010C03, 0, 0],
    "remote": [0x21010C02, 0, 1],
}

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
_transfer_proc: subprocess.Popen | None = None

import chassis_pb2  # noqa: E402
import std_msgs_pb2  # noqa: E402
from quadruped_mcp import SetPosture_Request, SetPosture_Response  # noqa: E402

# ── heartbeat state ───────────────────────────────────────────────────────
_hb_thread: "threading.Thread | None" = None
_hb_running = False
HEARTBEAT_INTERVAL = 0.2
HEARTBEAT_CMD = [0x21040001, 0, 0]


# ── transfer ROS2 subprocess management ───────────────────────────────────
def _spawn_transfer(cfg: dict) -> None:
    """Spawn transfer ROS2 nodes (jetson2motion, jetson2app, sensor_checker).

    The transfer package bridges UDP ↔ ROS2 for Lite3 motion host
    communication. See ``src/Lite3_ROS/src/transfer/launch/``.
    """
    global _transfer_proc
    env = dict(os.environ)
    # Pass cfg as env vars for the subprocess (launch file reads ROS params;
    # env vars are available for future extensibility).
    env["TRANSFER_TARGET_IP"] = str(cfg.get("target_ip", "192.168.1.120"))
    env["TRANSFER_TARGET_PORT"] = str(cfg.get("target_port", "43893"))
    env["TRANSFER_LOCAL_PORT"] = str(cfg.get("local_port", "43897"))
    env["TRANSFER_APP_PORT"] = str(cfg.get("app_port", "43899"))
    env["TRANSFER_ODOM_TOPIC"] = str(cfg.get("odom_topic", "/odom"))
    env["TRANSFER_PUBLISH_ODOM_TF"] = (
        "true" if bool(cfg.get("publish_odom_tf", True)) else "false"
    )

    log_path = _pkg_root / "rbnx-build" / "data" / "transfer.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    env["TRANSFER_APP_LOG_PATH"] = str(log_path.with_name("nx2app.log"))
    log_fh = open(log_path, "ab", buffering=0)
    log.info("spawning transfer: target_ip=%s target_port=%s odom_topic=%s publish_odom_tf=%s → %s",
             env["TRANSFER_TARGET_IP"], env["TRANSFER_TARGET_PORT"],
             env["TRANSFER_ODOM_TOPIC"], env["TRANSFER_PUBLISH_ODOM_TF"], log_path)
    _transfer_proc = subprocess.Popen(
        ["ros2", "launch", "transfer", "transfer_launch.py"],
        env=env,
        stdout=log_fh, stderr=log_fh,
        start_new_session=True,
    )


def _kill_transfer() -> None:
    """Kill the transfer subprocess group (SIGTERM → SIGKILL fallback)."""
    p = _transfer_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── UDP heartbeat ─────────────────────────────────────────────────────────
def _send_udp(cmd1: int, cmd2: int, cmd3: int) -> None:
    """Send a single 3-uint32_t UDP packet to the robot."""
    payload = struct.pack("<III", cmd1, cmd2, cmd3)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(payload, (ROBOT_IP, ROBOT_PORT))
    sock.close()


def _heartbeat_loop() -> None:
    """Background thread: send heartbeat every HEARTBEAT_INTERVAL seconds."""
    global _hb_running
    seq = 0
    while _hb_running:
        try:
            _send_udp(*HEARTBEAT_CMD)
            seq += 1
            log.debug("heartbeat #%d sent", seq)
        except Exception as e:
            log.warning("heartbeat error: %s", e)
        time.sleep(HEARTBEAT_INTERVAL)


def _start_heartbeat() -> None:
    """Launch the heartbeat background thread."""
    global _hb_thread, _hb_running
    if _hb_running:
        return
    _hb_running = True
    _hb_thread = threading.Thread(target=_heartbeat_loop, daemon=True)
    _hb_thread.start()
    log.info("heartbeat started (interval=%.1fs)", HEARTBEAT_INTERVAL)


def _stop_heartbeat() -> None:
    """Stop the heartbeat background thread."""
    global _hb_running, _hb_thread
    _hb_running = False
    if _hb_thread is not None:
        _hb_thread.join(timeout=1.0)
        _hb_thread = None
    log.info("heartbeat stopped")


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


def send_cmd(name: str):
    if name not in cmd_dict:
        log.error("Unknown command '%s'. Valid options: %s", name, list(cmd_dict.keys()))
        return
    cmd1, cmd2, cmd3 = cmd_dict[name]
    _send_udp(cmd1, cmd2, cmd3)
    log.info("cmd '%s' sent → %s:%s", name, ROBOT_IP, ROBOT_PORT)

@lite3_quadruped.mcp("robonix/primitive/chassis/posture")
def set_posture(req: SetPosture_Request) -> SetPosture_Response:
    """Set the Lite3 quadruped to a named posture.
    posture_name is a string like "stand", "sit", "zero", "init", etc. 
    you need to init be initialized before sending any commands.
    The driver maps each name to the
    corresponding servo-angle table and sends the joint commands."""
    
    send_cmd(req.posture_name)
    time.sleep(1.0)  # crude delay to allow the posture change to take effect before accepting the next command

    return SetPosture_Response(
        success=True,
        message=f"posture '{req.posture_name}' accepted (stub)",
    )


@lite3_quadruped.on_init
def init(cfg):
    """INACTIVE: spawn transfer ROS2 nodes, create publisher, wait for odom."""
    global cmd_vel_pub

    odom_topic = cfg.get("odom_topic") or os.environ.get("LITE3_ODOM_TOPIC", "/odom")
    twist_in_topic = cfg.get("twist_in_topic") or os.environ.get("LITE3_CMD_VEL_TOPIC", "/cmd_vel")

    # 1. Spawn transfer ROS2 nodes (jetson2motion, jetson2app, sensor_checker)
    #    These bridge UDP ↔ ROS2, producing /leg_odom2, /imu/data, /joint_states
    #    and consuming /cmd_vel.
    try:
        _spawn_transfer(cfg)
    except Exception as e:
        _kill_transfer()
        return Err(f"spawn transfer failed: {e}")

    from geometry_msgs.msg import Twist  # type: ignore

    cmd_vel_pub = lite3_quadruped.create_publisher(
        "robonix/primitive/chassis/twist_in",
        topic=twist_in_topic,
        msg_type=Twist,
        qos="reliable",
        declare=False,
    )
    lite3_quadruped.declare_ros2_topic(
        "robonix/primitive/chassis/twist_in",
        twist_in_topic,
        qos="reliable",
    )

    from nav_msgs.msg import Odometry  # type: ignore
    lite3_quadruped.create_subscription(
        "robonix/primitive/chassis/odom",
        topic=odom_topic,
        msg_type=Odometry,
        callback=_on_odom,
        qos="best_effort",
        declare=False,
    )
    lite3_quadruped.declare_ros2_topic(
        "robonix/primitive/chassis/odom",
        odom_topic,
        qos="reliable",
    )

    # 2. Wait for the odometry topic that transfer should produce
    if not lite3_quadruped.wait_for_topic(odom_topic, "nav_msgs/msg/Odometry", 15.0):
        _kill_transfer()
        return Deferred(f"ROS2 odom topic {odom_topic} not available")

    # 3. Start UDP heartbeat to keep the robot alive
    _start_heartbeat()

    # 4. Init sequence: init → zero.  The robot requires init first to
    #    unlock the motor controllers, then zero to calibrate joint
    #    positions before accepting motion commands.
    send_cmd("init")
    time.sleep(2.0)

    return Ok()


@lite3_quadruped.on_shutdown
def shutdown():
    """Clean up transfer subprocess on primitive shutdown."""
    _stop_heartbeat()
    _kill_transfer()
    return Ok()


if __name__ == "__main__":
    lite3_quadruped.run()
