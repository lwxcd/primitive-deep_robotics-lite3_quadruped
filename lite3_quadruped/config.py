"""Configuration parsing and safety validation for Lite3 backends."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Mapping


class ConfigError(ValueError):
    pass


HARDWARE_ONLY_KEYS = ("target_ip", "target_port", "local_port", "app_port")


@dataclass(frozen=True)
class RuntimeConfig:
    backend: str
    odom_topic: str
    twist_in_topic: str
    sentinel_timeout_s: float
    raw: dict[str, Any]


def parse_config(source: Mapping[str, Any] | None) -> RuntimeConfig:
    raw = dict(source or {})
    backend = str(os.environ.get(
        "LITE3_BACKEND", raw.get("backend", "hardware"))).strip().lower()
    if backend not in {"hardware", "simulation"}:
        raise ConfigError("backend must be 'hardware' or 'simulation'")

    if backend == "simulation":
        forbidden = [
            key for key in HARDWARE_ONLY_KEYS
            if key in raw and raw[key] not in (None, "")
        ]
        if forbidden:
            raise ConfigError(
                "simulation backend refuses hardware network settings: "
                + ", ".join(sorted(forbidden))
            )

    odom_default = "/odom"
    odom_topic = str(
        raw.get("odom_topic")
        or os.environ.get("LITE3_ODOM_TOPIC", odom_default)
    )
    twist_in_topic = str(
        raw.get("twist_in_topic")
        or os.environ.get("LITE3_CMD_VEL_TOPIC", "/cmd_vel")
    )
    timeout = float(raw.get("sentinel_timeout_s", 30.0))
    if timeout <= 0:
        raise ConfigError("sentinel_timeout_s must be positive")

    return RuntimeConfig(
        backend=backend,
        odom_topic=odom_topic,
        twist_in_topic=twist_in_topic,
        sentinel_timeout_s=timeout,
        raw=raw,
    )
