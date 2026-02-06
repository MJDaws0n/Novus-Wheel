from __future__ import annotations


class NovusWheelError(RuntimeError):
    """Base error for the Novus Wheel app."""


class CriticalWheelError(NovusWheelError):
    """A critical error that must stop motor output and exit."""


class BackendUnavailableError(CriticalWheelError):
    """Raised when the requested virtual device backend cannot be used."""
