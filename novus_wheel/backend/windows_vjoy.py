from __future__ import annotations

import ctypes
import queue
import sys
from dataclasses import dataclass

from .base import FfbCommand, VirtualWheelBackend, WheelAxisState
from ..errors import BackendUnavailableError, CriticalWheelError
from ..ffb.effects import ParsedEffects, clamp


# vJoy axis constants
HID_USAGE_X = 0x30


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _norm_to_vjoy_axis(x: float) -> int:
    # vJoy SetAxis uses 0..0x8000 (typically). Center at 0x4000.
    xn = _clamp(float(x), -1.0, 1.0)
    return int(round((xn + 1.0) * 0x4000))


# ctypes structs used by vJoy FFB helper functions
class FFB_DATA(ctypes.Structure):
    _fields_ = [("size", ctypes.c_uint), ("cmd", ctypes.c_uint), ("data", ctypes.c_ubyte * 1)]


class FFB_EFF_CONST(ctypes.Structure):
    _fields_ = [("Mag", ctypes.c_short)]


class FFB_EFF_SPRING(ctypes.Structure):
    _fields_ = [
        ("Offset", ctypes.c_short),
        ("Coef", ctypes.c_short),
        ("DeadBand", ctypes.c_ushort),
        ("Sat", ctypes.c_ushort),
    ]


class FFB_EFF_DAMPER(ctypes.Structure):
    _fields_ = [
        ("Offset", ctypes.c_short),
        ("Coef", ctypes.c_short),
        ("DeadBand", ctypes.c_ushort),
        ("Sat", ctypes.c_ushort),
    ]


@dataclass
class _VJoyFns:
    dll: ctypes.WinDLL

    vJoyEnabled: ctypes._CFuncPtr
    AcquireVJD: ctypes._CFuncPtr
    RelinquishVJD: ctypes._CFuncPtr
    ResetVJD: ctypes._CFuncPtr
    SetAxis: ctypes._CFuncPtr

    FfbRegisterGenCB: ctypes._CFuncPtr
    Ffb_h_Type: ctypes._CFuncPtr
    Ffb_h_DeviceID: ctypes._CFuncPtr
    Ffb_h_EffectType: ctypes._CFuncPtr
    FfbGetEffConst: ctypes._CFuncPtr
    FfbGetEffSpring: ctypes._CFuncPtr
    FfbGetEffDamper: ctypes._CFuncPtr


class VJoyWheelBackend(VirtualWheelBackend):
    """Windows backend using vJoy.

    Requires vJoy installed and configured with at least one device.

    Supports:
    - Steering axis via SetAxis
    - Force feedback via vJoy FFB callback (constant/spring/damper)
    """

    def __init__(self, *, device_id: int = 1, name: str = "Novus Wheel"):
        self._device_id = int(device_id)
        self._name = name

        self._fns: _VJoyFns | None = None
        self._cb = None
        self._ffb_q: "queue.Queue[ParsedEffects]" = queue.Queue(maxsize=8)
        self._effects = ParsedEffects()

    def _load(self) -> _VJoyFns:
        if sys.platform != "win32":
            raise BackendUnavailableError("vJoy backend is Windows-only")

        try:
            dll = ctypes.WinDLL("vJoyInterface.dll")
        except Exception as exc:
            raise BackendUnavailableError(
                "Could not load vJoyInterface.dll. Install vJoy and ensure its DLL is on PATH."
            ) from exc

        def fn(name: str, restype, argtypes):
            f = getattr(dll, name)
            f.restype = restype
            f.argtypes = argtypes
            return f

        # Basic joystick I/O
        vJoyEnabled = fn("vJoyEnabled", ctypes.c_bool, [])
        AcquireVJD = fn("AcquireVJD", ctypes.c_bool, [ctypes.c_uint])
        RelinquishVJD = fn("RelinquishVJD", None, [ctypes.c_uint])
        ResetVJD = fn("ResetVJD", ctypes.c_bool, [ctypes.c_uint])
        SetAxis = fn("SetAxis", ctypes.c_bool, [ctypes.c_long, ctypes.c_uint, ctypes.c_uint])

        # FFB helpers
        # Callback: void __stdcall cb(PVOID data, PVOID userData)
        CB = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
        FfbRegisterGenCB = fn("FfbRegisterGenCB", None, [CB, ctypes.c_void_p])

        Ffb_h_Type = fn("Ffb_h_Type", ctypes.c_bool, [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)])
        Ffb_h_DeviceID = fn("Ffb_h_DeviceID", ctypes.c_bool, [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)])
        Ffb_h_EffectType = fn("Ffb_h_EffectType", ctypes.c_bool, [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)])

        FfbGetEffConst = fn("FfbGetEffConst", ctypes.c_bool, [ctypes.c_void_p, ctypes.POINTER(FFB_EFF_CONST)])
        FfbGetEffSpring = fn("FfbGetEffSpring", ctypes.c_bool, [ctypes.c_void_p, ctypes.POINTER(FFB_EFF_SPRING)])
        FfbGetEffDamper = fn("FfbGetEffDamper", ctypes.c_bool, [ctypes.c_void_p, ctypes.POINTER(FFB_EFF_DAMPER)])

        return _VJoyFns(
            dll=dll,
            vJoyEnabled=vJoyEnabled,
            AcquireVJD=AcquireVJD,
            RelinquishVJD=RelinquishVJD,
            ResetVJD=ResetVJD,
            SetAxis=SetAxis,
            FfbRegisterGenCB=FfbRegisterGenCB,
            Ffb_h_Type=Ffb_h_Type,
            Ffb_h_DeviceID=Ffb_h_DeviceID,
            Ffb_h_EffectType=Ffb_h_EffectType,
            FfbGetEffConst=FfbGetEffConst,
            FfbGetEffSpring=FfbGetEffSpring,
            FfbGetEffDamper=FfbGetEffDamper,
        )

    def open(self) -> None:
        fns = self._load()
        if not bool(fns.vJoyEnabled()):
            raise BackendUnavailableError("vJoy driver is not enabled")

        if not bool(fns.AcquireVJD(self._device_id)):
            raise BackendUnavailableError(f"Failed to acquire vJoy device {self._device_id}. Is it configured/unused?")

        if not bool(fns.ResetVJD(self._device_id)):
            fns.RelinquishVJD(self._device_id)
            raise BackendUnavailableError(f"Failed to reset vJoy device {self._device_id}")

        # Register FFB callback
        CB = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)

        def _cb(data_ptr: ctypes.c_void_p, user_ptr: ctypes.c_void_p) -> None:
            try:
                self._on_ffb_packet(fns, data_ptr)
            except Exception:
                # Never throw from callback.
                return

        self._cb = CB(_cb)
        fns.FfbRegisterGenCB(self._cb, None)

        self._fns = fns

    def close(self) -> None:
        if self._fns is None:
            return
        try:
            # Center axis on close
            try:
                self._fns.SetAxis(_norm_to_vjoy_axis(0.0), self._device_id, HID_USAGE_X)
            except Exception:
                pass
        finally:
            try:
                self._fns.RelinquishVJD(self._device_id)
            finally:
                self._fns = None
                self._cb = None

    def update_axis(self, state: WheelAxisState) -> None:
        if self._fns is None:
            return
        v = _norm_to_vjoy_axis(state.steering)
        ok = bool(self._fns.SetAxis(v, self._device_id, HID_USAGE_X))
        if not ok:
            raise CriticalWheelError("vJoy SetAxis failed")

    def _on_ffb_packet(self, fns: _VJoyFns, data_ptr: ctypes.c_void_p) -> None:
        # Filter by device id.
        dev = ctypes.c_uint(0)
        if bool(fns.Ffb_h_DeviceID(data_ptr, ctypes.byref(dev))) and int(dev.value) != self._device_id:
            return

        eff_type = ctypes.c_uint(0)
        if not bool(fns.Ffb_h_EffectType(data_ptr, ctypes.byref(eff_type))):
            return

        # vJoy effect type constants (subset). These values come from vJoy SDK.
        # They are stable across SDK versions.
        ET_CONST = 0x01
        ET_SPRNG = 0x03
        ET_DAMPER = 0x04

        updated = False

        if int(eff_type.value) == ET_CONST:
            const = FFB_EFF_CONST()
            if bool(fns.FfbGetEffConst(data_ptr, ctypes.byref(const))):
                # Mag is signed short; vJoy scales commonly to [-10000,10000]
                self._effects.constant = clamp(float(const.Mag) / 10000.0, -1.0, 1.0)
                updated = True

        elif int(eff_type.value) == ET_SPRNG:
            spr = FFB_EFF_SPRING()
            if bool(fns.FfbGetEffSpring(data_ptr, ctypes.byref(spr))):
                self._effects.spring_center = clamp(float(spr.Offset) / 10000.0, -1.0, 1.0)
                self._effects.spring_k = clamp(abs(float(spr.Coef)) / 10000.0, 0.0, 1.0)
                updated = True

        elif int(eff_type.value) == ET_DAMPER:
            dmp = FFB_EFF_DAMPER()
            if bool(fns.FfbGetEffDamper(data_ptr, ctypes.byref(dmp))):
                self._effects.damper_b = clamp(abs(float(dmp.Coef)) / 10000.0, 0.0, 1.0)
                updated = True

        if updated:
            try:
                self._ffb_q.put_nowait(ParsedEffects(
                    constant=self._effects.constant,
                    spring_k=self._effects.spring_k,
                    spring_center=self._effects.spring_center,
                    damper_b=self._effects.damper_b,
                ))
            except queue.Full:
                # Drop oldest by clearing one then adding.
                try:
                    _ = self._ffb_q.get_nowait()
                except Exception:
                    pass
                try:
                    self._ffb_q.put_nowait(self._effects)
                except Exception:
                    pass

    def poll_ffb(self) -> FfbCommand | None:
        # Consume all queued updates and keep the latest.
        latest: ParsedEffects | None = None
        while True:
            try:
                latest = self._ffb_q.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self._effects = latest
        # Return the current constant force as a generic command; the app will
        # combine spring/damper using real pos/vel.
        return FfbCommand(force=self._effects.constant)

    @property
    def effects(self) -> ParsedEffects:
        return self._effects
