from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..ffb.effects import EffectState


@dataclass(frozen=True)
class WheelAxisState:
    # Normalized [-1, +1]
    steering: float


@dataclass(frozen=True)
class FfbCommand:
    """A single aggregated FFB command.

    This is not a raw DirectInput effect; it is the result of parsing/combining
    effects into a simple force model.
    """

    # Normalized [-1, +1] where +1 means max positive force.
    force: float


class VirtualWheelBackend:
    """Backend interface for a virtual wheel.

    Implementations:
    - Windows: vJoy axis output + vJoy FFB input.
    - macOS: SDL2 virtual joystick axis output; FFB may be unavailable.
    """

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def update_axis(self, state: WheelAxisState) -> None:
        raise NotImplementedError

    def poll_ffb(self) -> FfbCommand | None:
        """Return the latest aggregated FFB force, if any."""

        return None

    @property
    def effect_state(self) -> EffectState | None:
        """Return the per-slot FFB effect state, if available."""

        return None
