from __future__ import annotations

import sys
from dataclasses import dataclass

from .base import FfbCommand, VirtualWheelBackend, WheelAxisState
from ..errors import BackendUnavailableError


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class _SdlState:
    device_index: int
    joystick: object


class SdlVirtualWheelBackend(VirtualWheelBackend):
    """macOS-compatible backend using SDL2 virtual joystick.

    Notes:
    - This is *not* a kernel-level HID device; games must use SDL to see it.
    - Force feedback is not implemented here.

    This is still useful for development and for SDL-aware titles.
    """

    def __init__(self, *, name: str = "Novus Wheel"):
        self._name = name
        self._sdl = None
        self._state: _SdlState | None = None

    def open(self) -> None:
        try:
            import sdl2  # type: ignore[import-not-found]
            import sdl2.ext  # type: ignore[import-not-found]
        except Exception as exc:
            raise BackendUnavailableError(
                "SDL2 backend requires PySDL2 (`pip install pysdl2 pysdl2-dll`)"
            ) from exc

        self._sdl = sdl2

        if sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_HAPTIC) != 0:
            raise BackendUnavailableError(f"SDL_Init failed: {sdl2.SDL_GetError().decode('utf-8', 'ignore')}")

        # Create a virtual joystick with one steering axis.
        # SDL_JOYSTICK_TYPE_WHEEL exists in newer SDL; fall back to GAMECONTROLLER.
        joy_type = getattr(sdl2, "SDL_JOYSTICK_TYPE_WHEEL", getattr(sdl2, "SDL_JOYSTICK_TYPE_GAMECONTROLLER", 1))

        attach = getattr(sdl2, "SDL_JoystickAttachVirtual", None)
        if attach is None:
            raise BackendUnavailableError("SDL2 is too old; missing SDL_JoystickAttachVirtual")

        # Signature: (type, naxes, nbuttons, nhats) -> device_index
        device_index = int(attach(joy_type, 1, 0, 0))
        if device_index < 0:
            raise BackendUnavailableError(
                f"SDL_JoystickAttachVirtual failed: {sdl2.SDL_GetError().decode('utf-8', 'ignore')}"
            )

        joy = sdl2.SDL_JoystickOpen(device_index)
        if not joy:
            raise BackendUnavailableError(
                f"SDL_JoystickOpen failed: {sdl2.SDL_GetError().decode('utf-8', 'ignore')}"
            )

        self._state = _SdlState(device_index=device_index, joystick=joy)

    def close(self) -> None:
        if self._sdl is None:
            return
        try:
            if self._state is not None:
                try:
                    self._sdl.SDL_JoystickClose(self._state.joystick)
                except Exception:
                    pass
                detach = getattr(self._sdl, "SDL_JoystickDetachVirtual", None)
                if detach is not None:
                    try:
                        detach(int(self._state.device_index))
                    except Exception:
                        pass
            self._sdl.SDL_Quit()
        finally:
            self._sdl = None
            self._state = None

    def update_axis(self, state: WheelAxisState) -> None:
        if self._sdl is None or self._state is None:
            return

        sdl2 = self._sdl
        # SDL axis is int16 [-32768, 32767]
        v = int(round(_clamp(state.steering, -1.0, 1.0) * 32767.0))

        set_axis = getattr(sdl2, "SDL_JoystickSetVirtualAxis", None)
        if set_axis is None:
            return
        # axis 0
        set_axis(self._state.joystick, 0, v)

        # Pump events to keep SDL happy.
        try:
            sdl2.SDL_JoystickUpdate()
        except Exception:
            pass

    def poll_ffb(self) -> FfbCommand | None:
        # No force feedback on this backend.
        return None
