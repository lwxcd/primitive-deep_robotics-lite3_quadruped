from __future__ import annotations

import os
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from lite3_quadruped.backends import create_backend
from lite3_quadruped.backends.hardware import COMMANDS, HEARTBEAT, HardwareBackend
from lite3_quadruped.backends.simulation import SimulationBackend
from lite3_quadruped.config import ConfigError, parse_config


class ConfigTests(unittest.TestCase):
    def test_hardware_is_backward_compatible_default(self):
        with patch.dict(os.environ, {}, clear=True):
            config = parse_config({})
        self.assertEqual(config.backend, "hardware")
        self.assertEqual(config.odom_topic, "/odom")

    def test_simulation_defaults_to_standard_odom(self):
        with patch.dict(os.environ, {}, clear=True):
            config = parse_config({"backend": "simulation"})
        self.assertEqual(config.backend, "simulation")
        self.assertEqual(config.odom_topic, "/odom")

    def test_simulation_rejects_every_hardware_network_key(self):
        for key in ("target_ip", "target_port", "local_port", "app_port"):
            with self.subTest(key=key), patch.dict(os.environ, {}, clear=True):
                with self.assertRaisesRegex(ConfigError, key):
                    parse_config({"backend": "simulation", key: "unsafe"})

    def test_environment_can_select_backend(self):
        with patch.dict(os.environ, {"LITE3_BACKEND": "simulation"}, clear=True):
            config = parse_config({})
        self.assertEqual(config.backend, "simulation")


class BackendTests(unittest.TestCase):
    def simulation(self) -> SimulationBackend:
        config = parse_config({
            "backend": "simulation",
            "sentinel_timeout_s": 1.0,
        })
        backend = create_backend(config, Path("/unused"))
        self.assertIsInstance(backend, SimulationBackend)
        return backend

    @patch("lite3_quadruped.backends.simulation.subprocess.run")
    def test_simulation_postures_map_to_services(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, "success=True", "")
        backend = self.simulation()
        expected = {
            "stand": "/lite3_sim/stand",
            "sit": "/lite3_sim/lie_down",
            "reset": "/lite3_sim/reset",
            "emergency_stop": "/lite3_sim/emergency_stop",
        }
        for posture, service in expected.items():
            with self.subTest(posture=posture):
                success, _ = backend.set_posture(posture)
                self.assertTrue(success)
                self.assertEqual(run.call_args.args[0][3], service)

    @patch("lite3_quadruped.backends.simulation.subprocess.run")
    def test_simulation_discovery_checks_topic_type(self, run):
        run.return_value = subprocess.CompletedProcess(
            [], 0, "nav_msgs/msg/Odometry\n", "")
        backend = self.simulation()
        topic = backend.required_topics()[1]
        self.assertTrue(backend.wait_for_topic(None, topic))
        self.assertEqual(run.call_args.args[0], ["ros2", "topic", "type", "/odom"])

    @patch("lite3_quadruped.backends.simulation.subprocess.run")
    def test_simulation_sentinel_reads_each_required_topic(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, "message", "")
        backend = self.simulation()
        backend.verify_activity()
        topics = [call.args[0][4] for call in run.call_args_list]
        self.assertEqual(topics, ["/clock", "/odom", "/joint_states"])

    def test_hardware_keeps_manifest_network_values(self):
        config = parse_config({
            "backend": "hardware",
            "target_ip": "192.0.2.10",
            "target_port": 1234,
        })
        backend = create_backend(config, Path("/tmp/package"))
        self.assertIsInstance(backend, HardwareBackend)
        self.assertEqual(backend.target_ip, "192.0.2.10")
        self.assertEqual(backend.target_port, 1234)

    def hardware(self, raw=None, package_root=Path("/tmp/package")):
        config = parse_config({"backend": "hardware", **(raw or {})})
        backend = create_backend(config, package_root)
        self.assertIsInstance(backend, HardwareBackend)
        return backend

    def test_hardware_defaults_match_original_driver(self):
        with patch.dict(os.environ, {}, clear=True):
            backend = self.hardware()
        self.assertEqual(backend.target_ip, "192.168.1.120")
        self.assertEqual(backend.target_port, 43893)
        self.assertEqual(backend.local_port, 43897)
        self.assertEqual(backend.app_port, 43899)
        self.assertEqual(backend.config.odom_topic, "/odom")
        self.assertEqual(backend.config.twist_in_topic, "/cmd_vel")
        self.assertTrue(backend.publish_odom_tf)

    @patch("lite3_quadruped.backends.hardware.subprocess.Popen")
    def test_hardware_prepare_preserves_transfer_launch_and_environment(self, popen):
        popen.return_value.poll.return_value = 0
        with tempfile.TemporaryDirectory() as directory:
            backend = self.hardware({
                "target_ip": "192.0.2.20",
                "target_port": 1001,
                "local_port": 1002,
                "app_port": 1003,
                "odom_topic": "/robot_odom",
                "publish_odom_tf": False,
            }, Path(directory))
            backend.prepare()
            args, kwargs = popen.call_args
            self.assertEqual(args[0], [
                "ros2", "launch", "transfer", "transfer_launch.py"])
            self.assertTrue(kwargs["start_new_session"])
            env = kwargs["env"]
            self.assertEqual(env["TRANSFER_TARGET_IP"], "192.0.2.20")
            self.assertEqual(env["TRANSFER_TARGET_PORT"], "1001")
            self.assertEqual(env["TRANSFER_LOCAL_PORT"], "1002")
            self.assertEqual(env["TRANSFER_APP_PORT"], "1003")
            self.assertEqual(env["TRANSFER_ODOM_TOPIC"], "/robot_odom")
            self.assertEqual(env["TRANSFER_PUBLISH_ODOM_TF"], "false")
            backend.shutdown()

    def test_hardware_waits_for_configured_odometry(self):
        backend = self.hardware({
            "odom_topic": "/robot_odom", "sentinel_timeout_s": 17.0})
        capability = MagicMock()
        capability.wait_for_topic.return_value = True
        topic = backend.required_topics()[0]
        self.assertTrue(backend.wait_for_topic(capability, topic))
        capability.wait_for_topic.assert_called_once_with(
            "/robot_odom", "nav_msgs/msg/Odometry", 17.0)

    @patch("lite3_quadruped.backends.hardware.time.sleep")
    @patch("lite3_quadruped.backends.hardware.threading.Thread")
    def test_hardware_activate_starts_heartbeat_and_sends_init(self, thread, sleep):
        backend = self.hardware()
        with patch.object(backend, "_send_named") as send_named:
            backend.activate()
        thread.return_value.start.assert_called_once_with()
        send_named.assert_called_once_with("init")
        sleep.assert_called_once_with(2.0)

    @patch("lite3_quadruped.backends.hardware.time.sleep")
    @patch("lite3_quadruped.backends.hardware.socket.socket")
    def test_hardware_posture_uses_configured_motion_host(self, socket, sleep):
        backend = self.hardware({"target_ip": "192.0.2.30", "target_port": 4321})
        udp_socket = socket.return_value.__enter__.return_value
        success, _ = backend.set_posture("stand")
        self.assertTrue(success)
        udp_socket.sendto.assert_called_once_with(
            struct.pack("<III", *COMMANDS["stand"]), ("192.0.2.30", 4321))
        sleep.assert_called_once_with(1.0)
        self.assertEqual(backend.set_posture("unsupported")[0], False)

    def test_hardware_heartbeat_sends_immediately(self):
        backend = self.hardware()
        backend.heartbeat_stop = MagicMock()
        backend.heartbeat_stop.is_set.side_effect = [False, True]
        backend.heartbeat_stop.wait.return_value = False
        with patch.object(backend, "_send_udp") as send_udp:
            backend._heartbeat_loop()
        send_udp.assert_called_once_with(HEARTBEAT)
        backend.heartbeat_stop.wait.assert_called_once_with(0.2)


if __name__ == "__main__":
    unittest.main()
