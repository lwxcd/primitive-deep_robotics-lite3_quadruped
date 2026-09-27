"""Lite3 backend factory."""

from __future__ import annotations

from pathlib import Path

from .base import Backend
from .hardware import HardwareBackend
from .simulation import SimulationBackend
from ..config import RuntimeConfig


def create_backend(config: RuntimeConfig, package_root: Path) -> Backend:
    if config.backend == "hardware":
        return HardwareBackend(config, package_root)
    if config.backend == "simulation":
        return SimulationBackend(config)
    raise AssertionError(f"unhandled backend: {config.backend}")


__all__ = ["Backend", "create_backend"]
