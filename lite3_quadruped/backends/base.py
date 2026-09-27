"""Backend contract shared by hardware and simulation implementations."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from ..config import RuntimeConfig


@dataclass(frozen=True)
class RequiredTopic:
    name: str
    msg_type: str


class Backend(ABC):
    def __init__(self, config: RuntimeConfig):
        self.config = config

    @property
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def prepare(self) -> None:
        """Start backend providers needed before ROS topic discovery."""
        raise NotImplementedError

    @abstractmethod
    def required_topics(self) -> tuple[RequiredTopic, ...]:
        raise NotImplementedError

    def wait_for_topic(self, capability, topic: RequiredTopic) -> bool:
        """Wait for one topic using the Robonix ROS backend by default."""
        return capability.wait_for_topic(
            topic.name, topic.msg_type, self.config.sentinel_timeout_s)

    def verify_activity(self) -> None:
        """Verify that discovered topics are actively publishing."""

    @abstractmethod
    def activate(self) -> None:
        """Run backend actions after all topic sentinels pass."""
        raise NotImplementedError

    @abstractmethod
    def set_posture(self, posture: str) -> tuple[bool, str]:
        raise NotImplementedError

    @abstractmethod
    def shutdown(self) -> None:
        raise NotImplementedError
