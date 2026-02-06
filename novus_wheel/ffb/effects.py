from __future__ import annotations

from dataclasses import dataclass


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class ParsedEffects:
    """Simplified effect model.

    All fields are normalized such that typical vJoy magnitudes map to [-1,+1].
    """

    constant: float = 0.0
    spring_k: float = 0.0  # force per normalized position
    spring_center: float = 0.0
    damper_b: float = 0.0  # force per normalized velocity

    def compute_force(self, *, pos: float, vel: float) -> float:
        f = 0.0
        f += self.constant
        f += -self.spring_k * (pos - self.spring_center)
        f += -self.damper_b * vel
        return clamp(f, -1.0, 1.0)
