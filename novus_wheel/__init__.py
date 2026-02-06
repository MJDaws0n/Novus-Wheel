"""Novus Wheel - ODrive-backed virtual steering wheel + force feedback.

Core goals:
- Read real wheel position/velocity from ODrive encoder.
- Present a virtual game controller axis for steering.
- Receive force-feedback (FFB) from games and drive motor torque.

Windows: vJoy backend (DirectInput FFB) - requires vJoy installed.
macOS: SDL2 virtual joystick backend (steering only; FFB limited).
"""

from __future__ import annotations

__all__ = ["__version__"]

__version__ = "0.1.0"
