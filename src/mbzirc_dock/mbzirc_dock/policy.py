from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from rclpy.time import Time


@dataclass
class PolicyState:
    time: Time
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    target: np.ndarray


@dataclass
class PolicyAction:
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray


class Policy(ABC):
    def __init__(self) -> None:
        self._is_canceled = False

    def cancel(self):
        self._is_canceled = True

    def is_canceled(self) -> bool:
        return self._is_canceled

    @abstractmethod
    def get_action(self, prev_state: PolicyState) -> PolicyAction:
        ...

class PolicyCanceledException(Exception):
    pass


class PolicyOutOfBoundsException(Exception):
    pass
