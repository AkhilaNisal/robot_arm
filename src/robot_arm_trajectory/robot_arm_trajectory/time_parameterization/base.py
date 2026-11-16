"""Common scalar path time parameterization profile."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class TimeScalingProfile:
    time: np.ndarray
    s: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray | None = None


class TimeParameterizer(ABC):
    @abstractmethod
    def generate(self, duration, num_samples, **kwargs) -> TimeScalingProfile:
        """Create a profile with path position and time derivatives."""
