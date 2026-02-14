from __future__ import annotations

from .base import VirtualWheelBackend, WheelAxisState


class NoneBackend(VirtualWheelBackend):
    """Backend that does not expose any virtual joystick.

    This is useful for running Novus Wheel as a standalone FFB / motor-control
    application without depending on vJoy/SDL or presenting a device to games.
    """

    def open(self) -> None:
        return

    def close(self) -> None:
        return

    def update_axis(self, state: WheelAxisState) -> None:
        return
