from __future__ import annotations

import ctypes
import os
import sys
import time
import math
from pathlib import Path
from dataclasses import dataclass

from .base import FfbCommand, VirtualWheelBackend, WheelAxisState
from ..errors import BackendUnavailableError, CriticalWheelError
from ..ffb.effects import EffectState, EffectSlot, clamp, ET_CONSTANT, ET_SPRING, ET_DAMPER, ET_FRICTION, ET_INERTIA


# vJoy axis constants
HID_USAGE_X = 0x30


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _norm_to_vjoy_axis(x: float) -> int:
    # vJoy SetAxis uses 0..0x8000 (typically). Center at 0x4000.
    xn = _clamp(float(x), -1.0, 1.0)
    # Some vJoy builds reject the top endpoint (0x8000). Clamp to 0x7FFF.
    v = int(round((xn + 1.0) * 0x4000))
    return 0 if v < 0 else 0x7FFF if v > 0x7FFF else v


# ctypes structs used by vJoy FFB helper functions
class FFB_DATA(ctypes.Structure):
    _fields_ = [("size", ctypes.c_uint), ("cmd", ctypes.c_uint), ("data", ctypes.c_ubyte * 1)]


class FFB_EFF_CONSTANT(ctypes.Structure):
    _fields_ = [("EffectBlockIndex", ctypes.c_ubyte), ("Magnitude", ctypes.c_long)]


class FFB_EFF_COND(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("isY", ctypes.c_int),
        ("CenterPointOffset", ctypes.c_long),
        ("PosCoeff", ctypes.c_long),
        ("NegCoeff", ctypes.c_long),
        ("PosSatur", ctypes.c_uint),
        ("NegSatur", ctypes.c_uint),
        ("DeadBand", ctypes.c_long),
    ]


class FFB_EFF_REPORT(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("_pad1", ctypes.c_ubyte * 3),
        ("EffectType", ctypes.c_uint),
        ("Duration", ctypes.c_ushort),
        ("TrigerRpt", ctypes.c_ushort),
        ("SamplePrd", ctypes.c_ushort),
        ("Gain", ctypes.c_ubyte),
        ("TrigerBtn", ctypes.c_ubyte),
    ]


class FFB_EFF_OP(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),
        ("_pad1", ctypes.c_ubyte * 3),
        ("EffectOp", ctypes.c_uint),
        ("LoopCount", ctypes.c_ubyte),
    ]


@dataclass
class _VJoyFns:
    dll: ctypes.WinDLL

    vJoyEnabled: ctypes._CFuncPtr
    AcquireVJD: ctypes._CFuncPtr
    RelinquishVJD: ctypes._CFuncPtr
    ResetVJD: ctypes._CFuncPtr
    SetAxis: ctypes._CFuncPtr

    # FFB helpers (all optional; some vJoyInterface.dll builds lack them).
    FfbRegisterGenCB: ctypes._CFuncPtr | None
    Ffb_h_Type: ctypes._CFuncPtr | None
    Ffb_h_DeviceID: ctypes._CFuncPtr | None
    Ffb_h_EBI: ctypes._CFuncPtr | None
    Ffb_h_Eff_Report: ctypes._CFuncPtr | None
    Ffb_h_Eff_Constant: ctypes._CFuncPtr | None
    Ffb_h_Eff_Cond: ctypes._CFuncPtr | None
    Ffb_h_EffOp: ctypes._CFuncPtr | None
    Ffb_h_DevCtrl: ctypes._CFuncPtr | None


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
        self._effect_state = EffectState()
        self._dll_dir_handles: list[object] = []

        self._effect_state.stale_timeout_s = float(
            os.environ.get("NOVUS_WHEEL_FFB_STALE_TIMEOUT_S", "2.0") or 2.0
        )

        self._vjoy_dll_loaded_from: str | None = None
        self._ffb_available: bool = False
        self._diag = os.environ.get("NOVUS_WHEEL_VJOY_DIAG", "").strip().lower() in {"1", "true", "yes", "on"}

    @staticmethod
    def _pe_arch(path: Path) -> str | None:
        """Best-effort PE architecture probe.

        Returns "x86" or "x64" when detected, else None.
        """

        try:
            with path.open("rb") as f:
                mz = f.read(64)
                if len(mz) < 64 or mz[:2] != b"MZ":
                    return None
                e_lfanew = int.from_bytes(mz[0x3C : 0x40], "little", signed=False)
                f.seek(e_lfanew)
                pe = f.read(24)
                if len(pe) < 24 or pe[:4] != b"PE\x00\x00":
                    return None
                machine = int.from_bytes(pe[4:6], "little", signed=False)

            if machine == 0x014C:
                return "x86"
            if machine == 0x8664:
                return "x64"
            return None
        except Exception:
            return None

    @staticmethod
    def _python_arch() -> str:
        # sys.maxsize is 2**31-1 on 32-bit, 2**63-1 on 64-bit
        return "x64" if sys.maxsize > 2**32 else "x86"

    @staticmethod
    def _iter_vjoy_candidate_dirs() -> list[Path]:
        candidates: list[Path] = []

        # 1) PATH entries
        for p in os.environ.get("PATH", "").split(os.pathsep):
            if not p:
                continue
            try:
                candidates.append(Path(p))
            except Exception:
                continue

        # 2) Common install roots
        for env_key in ("PROGRAMFILES", "PROGRAMFILES(X86)"):
            root = os.environ.get(env_key)
            if not root:
                continue
            base = Path(root) / "vJoy"
            # Some installers place DLLs directly under vJoy, others under arch folders.
            candidates.extend(
                [
                    base,
                    base / "x64",
                    base / "x86",
                    base / "bin",
                    base / "bin" / "x64",
                    base / "bin" / "x86",
                ]
            )

        # 3) Optional explicit hint
        for env_key in ("VJOY_DLL_DIR", "VJOY_HOME", "VJOY_INSTALL_DIR"):
            v = os.environ.get(env_key)
            if v:
                candidates.append(Path(v))

        # 4) Optional explicit full DLL path
        explicit_dll = os.environ.get("VJOY_INTERFACE_DLL")
        if explicit_dll:
            try:
                p = Path(explicit_dll)
                if p.suffix.lower() == ".dll":
                    candidates.append(p.parent)
            except Exception:
                pass

        # De-dupe while preserving order
        seen: set[str] = set()
        unique: list[Path] = []
        for c in candidates:
            try:
                key = str(c.resolve())
            except Exception:
                key = str(c)
            if key in seen:
                continue
            seen.add(key)
            unique.append(c)
        return unique

    def _try_load_vjoy_dll(self) -> ctypes.WinDLL:
        dll_name = "vJoyInterface.dll"

        # Python 3.8+ tightened Windows DLL search rules; PATH alone may not be used.
        add_dir = getattr(os, "add_dll_directory", None)

        def _dll_supports_ffb(loaded: ctypes.WinDLL) -> bool:
            # We need the callback registrar and at least one modern helper.
            try:
                _ = getattr(loaded, "FfbRegisterGenCB")
            except Exception:
                return False

            for n in ("Ffb_h_Eff_Constant", "Ffb_h_Eff_Cond", "Ffb_h_Eff_Report"):
                try:
                    _ = getattr(loaded, n)
                    return True
                except Exception:
                    continue

            return False

        # Highest priority: explicit DLL path override
        explicit_path = os.environ.get("VJOY_INTERFACE_DLL")
        if explicit_path:
            p = Path(explicit_path)
            if p.is_file():
                if add_dir is not None:
                    try:
                        handle = add_dir(str(p.parent))
                        self._dll_dir_handles.append(handle)
                    except Exception:
                        pass
                loaded = ctypes.WinDLL(str(p))
                self._vjoy_dll_loaded_from = str(p)
                self._ffb_available = _dll_supports_ffb(loaded)
                return loaded

        found_paths: list[Path] = []
        for d in self._iter_vjoy_candidate_dirs():
            try:
                dll_path = d / dll_name
                if dll_path.is_file():
                    found_paths.append(dll_path)
            except Exception:
                continue

        # Prefer loading by explicit full path if we can find it.
        load_errors: list[str] = []
        loaded_candidates: list[tuple[Path, ctypes.WinDLL, bool]] = []

        def _load_from_dir(dir_path: Path) -> None:
            if add_dir is None:
                return
            try:
                handle = add_dir(str(dir_path))
                self._dll_dir_handles.append(handle)
            except Exception:
                return

        # If we found the DLL in any directory, add those dirs first.
        for p in found_paths:
            _load_from_dir(p.parent)

        # Attempt explicit-path loads (most reliable)
        for p in found_paths:
            try:
                loaded = ctypes.WinDLL(str(p))
                loaded_candidates.append((p, loaded, _dll_supports_ffb(loaded)))
            except OSError as exc:
                arch = self._pe_arch(p)
                py_arch = self._python_arch()
                hint = ""
                if getattr(exc, "winerror", None) == 193 and arch is not None:
                    hint = f" (DLL is {arch}, Python is {py_arch} — arch mismatch)"
                load_errors.append(f"{p}: {exc}{hint}")
            except Exception as exc:
                load_errors.append(f"{p}: {exc}")

        # If we loaded multiple DLLs, prefer one that supports FFB.
        for p, loaded, has_ffb in loaded_candidates:
            if has_ffb:
                self._vjoy_dll_loaded_from = str(p)
                self._ffb_available = True
                return loaded

        # Otherwise fall back to the first one that loaded.
        if loaded_candidates:
            self._vjoy_dll_loaded_from = str(loaded_candidates[0][0])
            self._ffb_available = bool(loaded_candidates[0][2])
            return loaded_candidates[0][1]

        # Fall back to default name-based load (covers cases where DLL is elsewhere,
        # and where AddDllDirectory handles above made it discoverable).
        try:
            loaded = ctypes.WinDLL(dll_name)
            self._vjoy_dll_loaded_from = getattr(loaded, "_name", None) or dll_name
            self._ffb_available = _dll_supports_ffb(loaded)
            return loaded
        except OSError as exc:
            py_arch = self._python_arch()
            details = []
            if load_errors:
                details.append("Tried explicit DLL paths:")
                details.extend(f"  - {e}" for e in load_errors[:12])

            # Provide actionable hints for the two most common cases.
            hint_lines = [
                f"Python architecture: {py_arch}",
                "Common causes:",
                "  - vJoy is not installed / driver not present",
                "  - vJoyInterface.dll is 32-bit but you are running 64-bit Python (or vice versa)",
                "  - DLL is in a folder on PATH, but Python 3.8+ won't search PATH unless the folder is added via add_dll_directory",
            ]

            searched = self._iter_vjoy_candidate_dirs()
            searched_preview = [str(p) for p in searched[:25]]
            msg = (
                "Could not load vJoyInterface.dll. "
                "vJoy must be installed, and the DLL must be discoverable by the current Python process.\n"
                + "\n".join(hint_lines)
                + "\nSearched directories (first 25):\n  - "
                + "\n  - ".join(searched_preview)
            )
            if details:
                msg += "\n" + "\n".join(details)
            raise BackendUnavailableError(msg) from exc

    def _load(self) -> _VJoyFns:
        if sys.platform != "win32":
            raise BackendUnavailableError("vJoy backend is Windows-only")

        dll = self._try_load_vjoy_dll()

        def fn(name: str, restype, argtypes, *, optional: bool = False):
            try:
                f = getattr(dll, name)
            except AttributeError:
                if optional:
                    return None
                raise
            f.restype = restype
            f.argtypes = argtypes
            return f

        # Basic joystick I/O
        vJoyEnabled = fn("vJoyEnabled", ctypes.c_bool, [])
        AcquireVJD = fn("AcquireVJD", ctypes.c_bool, [ctypes.c_uint])
        RelinquishVJD = fn("RelinquishVJD", None, [ctypes.c_uint])
        ResetVJD = fn("ResetVJD", ctypes.c_bool, [ctypes.c_uint])
        SetAxis = fn("SetAxis", ctypes.c_bool, [ctypes.c_long, ctypes.c_uint, ctypes.c_uint])

        # FFB helpers (optional)
        CB = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
        FfbRegisterGenCB = fn("FfbRegisterGenCB", None, [CB, ctypes.c_void_p], optional=True)

        Ffb_h_Type = fn("Ffb_h_Type", ctypes.c_uint, [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)], optional=True)
        Ffb_h_DeviceID = fn("Ffb_h_DeviceID", ctypes.c_uint, [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)], optional=True)
        Ffb_h_EBI = fn("Ffb_h_EBI", ctypes.c_uint, [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)], optional=True)

        Ffb_h_Eff_Report = fn(
            "Ffb_h_Eff_Report", ctypes.c_uint, [ctypes.c_void_p, ctypes.POINTER(FFB_EFF_REPORT)], optional=True
        )
        Ffb_h_Eff_Constant = fn(
            "Ffb_h_Eff_Constant", ctypes.c_uint, [ctypes.c_void_p, ctypes.POINTER(FFB_EFF_CONSTANT)], optional=True
        )
        Ffb_h_Eff_Cond = fn("Ffb_h_Eff_Cond", ctypes.c_uint, [ctypes.c_void_p, ctypes.POINTER(FFB_EFF_COND)], optional=True)
        Ffb_h_EffOp = fn("Ffb_h_EffOp", ctypes.c_uint, [ctypes.c_void_p, ctypes.POINTER(FFB_EFF_OP)], optional=True)
        Ffb_h_DevCtrl = fn("Ffb_h_DevCtrl", ctypes.c_uint, [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)], optional=True)

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
            Ffb_h_EBI=Ffb_h_EBI,
            Ffb_h_Eff_Report=Ffb_h_Eff_Report,
            Ffb_h_Eff_Constant=Ffb_h_Eff_Constant,
            Ffb_h_Eff_Cond=Ffb_h_Eff_Cond,
            Ffb_h_EffOp=Ffb_h_EffOp,
            Ffb_h_DevCtrl=Ffb_h_DevCtrl,
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

        # Register FFB callback if the DLL supports it.
        has_any_getter = any([
            fns.Ffb_h_Eff_Constant is not None,
            fns.Ffb_h_Eff_Cond is not None,
            fns.Ffb_h_Eff_Report is not None,
        ])
        if fns.FfbRegisterGenCB is not None and has_any_getter:
            CB = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)

            def _cb(data_ptr: ctypes.c_void_p, user_ptr: ctypes.c_void_p) -> None:
                try:
                    self._on_ffb_packet(fns, data_ptr)
                except Exception:
                    # Never throw from callback.
                    return

            self._cb = CB(_cb)
            try:
                fns.FfbRegisterGenCB(self._cb, None)
            except Exception:
                # If registering fails, continue axis-only.
                self._cb = None
        else:
            # Axis-only mode; games will see the device but FFB cannot be read.
            self._cb = None

        if self._diag:
            try:
                where = self._vjoy_dll_loaded_from or getattr(fns.dll, "_name", None) or "(unknown)"
                ffb_exports = bool(fns.FfbRegisterGenCB is not None and has_any_getter)
                cb_ok = bool(self._cb is not None)
                print(f"[vJoy] dll={where} ffb_exports={ffb_exports} ffb_cb_registered={cb_ok}")
                if not ffb_exports:
                    print(
                        "[vJoy] No FFB exports in this DLL. Install an FFB-capable vJoy build or set VJOY_INTERFACE_DLL to the correct vJoyInterface.dll."
                    )
            except Exception:
                pass

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
                # Keep the add_dll_directory handles alive for as long as we're open;
                # then release them on close.
                for h in self._dll_dir_handles:
                    try:
                        close = getattr(h, "close", None)
                        if close is not None:
                            close()
                    except Exception:
                        pass
                self._dll_dir_handles.clear()

    def update_axis(self, state: WheelAxisState) -> None:
        if self._fns is None:
            return
        v = _norm_to_vjoy_axis(state.steering)
        ok = bool(self._fns.SetAxis(v, self._device_id, HID_USAGE_X))
        if ok:
            return

        # Do NOT call ResetVJD here: some games will stop reading the device until they
        # re-enumerate / regain focus (it can look like inputs "freeze" until Alt-Tab).
        # Instead: retry once, then attempt to re-acquire the device, then fail fast.
        try:
            ok2 = bool(self._fns.SetAxis(v, self._device_id, HID_USAGE_X))
        except Exception:
            ok2 = False
        if ok2:
            return

        try:
            _ = bool(self._fns.AcquireVJD(self._device_id))
        except Exception:
            pass
        try:
            ok3 = bool(self._fns.SetAxis(v, self._device_id, HID_USAGE_X))
        except Exception:
            ok3 = False
        if not ok3:
            raise CriticalWheelError(
                "vJoy SetAxis failed. Ensure the vJoy device is acquired and the X axis is enabled in vJoyConf."
            )

    def _on_ffb_packet(self, fns: _VJoyFns, data_ptr: ctypes.c_void_p) -> None:
        """Handle an incoming FFB packet from vJoy.

        Parses HID PID reports and updates per-slot effect state.
        """
        # Filter by device ID.
        if fns.Ffb_h_DeviceID is not None:
            dev = ctypes.c_uint(0)
            try:
                fns.Ffb_h_DeviceID(data_ptr, ctypes.byref(dev))
                if int(dev.value) != 0 and int(dev.value) != self._device_id:
                    return
            except Exception:
                pass

        # Determine packet type (HID PID report ID).
        pkt_type: int | None = None
        if fns.Ffb_h_Type is not None:
            pt = ctypes.c_uint(0)
            try:
                fns.Ffb_h_Type(data_ptr, ctypes.byref(pt))
                pkt_type = int(pt.value)
            except Exception:
                pass

        if pkt_type is None:
            return  # Cannot process without knowing the packet type.

        self._effect_state.touch()

        def _raw_payload_bytes() -> bytes | None:
            """Best-effort extraction of the HID report payload bytes.

            vJoy passes an FFB_DATA struct: {size, cmd, data[]}. We strip the 8-byte
            header and return the remaining data.
            """

            try:
                fb = ctypes.cast(data_ptr, ctypes.POINTER(FFB_DATA)).contents
                sz = int(getattr(fb, "size", 0) or 0)
                if sz <= 8 or sz > 4096:
                    return None
                raw = ctypes.string_at(data_ptr, sz)
                if not raw or len(raw) <= 8:
                    return None
                return bytes(raw[8:])
            except Exception:
                return None

        def _try_decode_set_effect_direction_deg(*, block_id: int, effect_type: int) -> tuple[float, int] | None:
            """Try to decode the direction field from a Set Effect (0x01) report.

            Many wheel titles encode constant-force sign in the direction field rather than magnitude.
            We decode a polar angle and map it to a sign via cos(theta): 0° => +, 180° => -.
            """

            payload = _raw_payload_bytes()
            if not payload:
                return None

            start = None
            if (
                len(payload) >= 3
                and payload[0] == 0x01
                and payload[1] == (block_id & 0xFF)
                and payload[2] == (effect_type & 0xFF)
            ):
                start = 0
            else:
                pat = bytes([(block_id & 0xFF), (effect_type & 0xFF)])
                i = payload.find(pat)
                if i >= 1 and payload[i - 1] == 0x01:
                    start = i - 1

            if start is None:
                return None

            # Heuristics:
            # - Direction is commonly 2 bytes, little-endian, in 0.01 degrees (0..35900)
            # - Some stacks lay out fields differently; scan a small window.
            # - Prefer values that look like multiples of 0.01° and near cardinal directions.

            best: tuple[float, int, float] | None = None  # (score, off, deg)

            win_start = max(0, start + 6)
            win_end = min(len(payload) - 1, start + 28)
            for off in range(win_start, win_end):
                if off + 1 >= len(payload):
                    continue
                d2 = int.from_bytes(payload[off : off + 2], "little", signed=False)
                if not (0 <= d2 <= 35900):
                    continue
                # Filter obvious noise: most real angles are multiples of 100 (whole degrees)
                # or multiples of 1 (0.01°). Accept all, but score whole-degree higher.
                deg = float(d2) / 100.0
                # Score by distance to nearest cardinal direction.
                card = min(abs(deg - 0.0), abs(deg - 90.0), abs(deg - 180.0), abs(deg - 270.0), abs(deg - 360.0))
                score = float(card)
                if d2 % 100 != 0:
                    score += 1.0
                if best is None or score < best[0]:
                    best = (score, off, deg)

            if best is not None and best[0] <= 10.0:
                return float(best[2]), int(best[1])

            # Fallback: 1-byte scaled direction at a typical offset.
            off1 = start + 12
            if len(payload) > off1:
                b = int(payload[off1]) & 0xFF
                return (float(b) / 255.0) * 360.0, int(off1)

            return None

        def _apply_direction_sign(slot: EffectSlot, *, direction_deg: float | None) -> None:
            if direction_deg is None:
                return
            try:
                slot.direction_sign = 1.0 if math.cos(math.radians(float(direction_deg))) >= 0.0 else -1.0
            except Exception:
                return

        # 0x01: Set Effect Report — defines effect type and gain for a block.
        if pkt_type == 0x01 and fns.Ffb_h_Eff_Report is not None:
            rep = FFB_EFF_REPORT()
            try:
                fns.Ffb_h_Eff_Report(data_ptr, ctypes.byref(rep))
                block_id = int(rep.EffectBlockIndex)
                slot = self._effect_state.get_or_create(block_id)
                slot.effect_type = int(rep.EffectType)
                # Gain is 0–255 → normalize to 0–1.
                gain_raw = int(rep.Gain) & 0xFF
                # Many games/drivers leave this field at 0 (unspecified). In practice this
                # should be treated as full-scale, otherwise all forces become zero.
                slot.gain = (float(gain_raw) / 255.0) if gain_raw != 0 else 1.0
                # Many games don't send an explicit Start; auto-play on creation.
                slot.playing = True

                dec = _try_decode_set_effect_direction_deg(block_id=block_id, effect_type=int(rep.EffectType))
                direction_deg = dec[0] if dec is not None else None
                direction_off = dec[1] if dec is not None else None
                _apply_direction_sign(slot, direction_deg=direction_deg)

                if direction_deg is not None and direction_off is not None:
                    self._effect_state.record_packet_debug(
                        f"FFB SetEffect: block={block_id} type=0x{int(rep.EffectType):02X} gain={slot.gain:.3f} gainRaw={gain_raw} dirDeg={direction_deg:.1f} dirOff={int(direction_off)} sign={slot.direction_sign:+.0f}"
                    )
                else:
                    self._effect_state.record_packet_debug(
                        f"FFB SetEffect: block={block_id} type=0x{int(rep.EffectType):02X} gain={slot.gain:.3f} gainRaw={gain_raw}"
                    )
            except Exception:
                pass
            return

        # 0x03: Set Condition Report — spring / damper / friction parameters.
        if pkt_type == 0x03 and fns.Ffb_h_Eff_Cond is not None:
            cond = FFB_EFF_COND()
            try:
                fns.Ffb_h_Eff_Cond(data_ptr, ctypes.byref(cond))
                block_id = int(cond.EffectBlockIndex)
                # We only care about the X axis (steering).
                if int(cond.isY) != 0:
                    return

                def _scale_for(v: int) -> float:
                    # DirectInput commonly uses 0..10000, but some stacks report 0..32767.
                    return 32767.0 if abs(int(v)) > 10000 else 10000.0

                c_center_raw = int(cond.CenterPointOffset)
                c_pos_coeff_raw = int(cond.PosCoeff)
                c_neg_coeff_raw = int(cond.NegCoeff)
                c_pos_sat_raw = int(cond.PosSatur)
                c_neg_sat_raw = int(cond.NegSatur)
                c_dead_raw = int(cond.DeadBand)

                slot = self._effect_state.get_or_create(block_id)
                slot.center = clamp(float(c_center_raw) / _scale_for(c_center_raw), -1.0, 1.0)
                # Our condition formula uses f = -coeff * offset, so coeff must be positive
                # (abs) to produce restoring/damping forces. The negative sign in the formula
                # handles direction; game-side sign conventions vary and would break this.
                slot.pos_coeff = clamp(abs(float(c_pos_coeff_raw)) / _scale_for(c_pos_coeff_raw), 0.0, 1.0)
                slot.neg_coeff = clamp(abs(float(c_neg_coeff_raw)) / _scale_for(c_neg_coeff_raw), 0.0, 1.0)

                # Also retain the signed interpretation. Some games send negative coefficients
                # and expect the driver to use them directly (without a leading minus).
                slot.pos_coeff_signed = clamp(float(c_pos_coeff_raw) / _scale_for(c_pos_coeff_raw), -1.0, 1.0)
                slot.neg_coeff_signed = clamp(float(c_neg_coeff_raw) / _scale_for(c_neg_coeff_raw), -1.0, 1.0)
                slot.cond_use_signed_coeff = bool(int(c_pos_coeff_raw) < 0 or int(c_neg_coeff_raw) < 0)

                if c_pos_sat_raw > 0:
                    slot.pos_sat = clamp(float(c_pos_sat_raw) / _scale_for(c_pos_sat_raw), 0.0, 1.0)
                else:
                    slot.pos_sat = 1.0

                if c_neg_sat_raw > 0:
                    slot.neg_sat = clamp(float(c_neg_sat_raw) / _scale_for(c_neg_sat_raw), 0.0, 1.0)
                else:
                    slot.neg_sat = 1.0

                slot.dead_band = clamp(abs(float(c_dead_raw)) / _scale_for(c_dead_raw), 0.0, 1.0)
                slot.playing = True

                self._effect_state.record_packet_debug(
                    "FFB SetCondition: "
                    f"block={block_id} isY={int(cond.isY)} "
                    f"center={slot.center:+.3f} dead={slot.dead_band:.3f} "
                    f"posCoeff={slot.pos_coeff:.3f} negCoeff={slot.neg_coeff:.3f} "
                    f"signedMode={int(bool(slot.cond_use_signed_coeff))} posCoeffS={slot.pos_coeff_signed:+.3f} negCoeffS={slot.neg_coeff_signed:+.3f} "
                    f"posSat={slot.pos_sat:.3f} negSat={slot.neg_sat:.3f}"
                    f" rawCenter={c_center_raw} rawDead={c_dead_raw} rawPosCoeff={c_pos_coeff_raw} rawNegCoeff={c_neg_coeff_raw}"
                )
            except Exception:
                pass
            return

        # 0x05: Set Constant Force Report.
        if pkt_type == 0x05 and fns.Ffb_h_Eff_Constant is not None:
            const = FFB_EFF_CONSTANT()
            try:
                fns.Ffb_h_Eff_Constant(data_ptr, ctypes.byref(const))
                block_id = int(const.EffectBlockIndex)
                slot = self._effect_state.get_or_create(block_id)
                # vJoy helper commonly provides a 16-bit signed magnitude in a wider integer.
                # If the value looks like an unsigned 16-bit (0..65535), sign-extend it.
                raw_u = int(const.Magnitude)
                raw_s = raw_u
                if 0 <= raw_u <= 0xFFFF and raw_u > 0x7FFF:
                    raw_s = raw_u - 0x10000

                mag = float(raw_s) / 10000.0
                if mag < 0.0:
                    slot.const_magnitude_is_signed = True
                slot.magnitude = clamp(float(mag), -1.0, 1.0)
                slot.playing = True

                self._effect_state.record_packet_debug(
                    f"FFB SetConstant: block={block_id} magnitude={slot.magnitude:+.3f} raw={raw_u} rawSigned={raw_s} dirSign={slot.direction_sign:+.0f} signed={int(bool(slot.const_magnitude_is_signed))}"
                )
            except Exception:
                pass
            return

        # 0x0A: Effect Operation Report (start / stop).
        if pkt_type == 0x0A and fns.Ffb_h_EffOp is not None:
            op = FFB_EFF_OP()
            try:
                fns.Ffb_h_EffOp(data_ptr, ctypes.byref(op))
                block_id = int(op.EffectBlockIndex)
                operation = int(op.EffectOp)
                if operation == 1:  # EFF_START
                    self._effect_state.start(block_id)
                    self._effect_state.record_packet_debug(f"FFB Op: START block={block_id}")
                elif operation == 2:  # EFF_SOLO
                    self._effect_state.start(block_id, solo=True)
                    self._effect_state.record_packet_debug(f"FFB Op: SOLO block={block_id}")
                elif operation == 3:  # EFF_STOP
                    self._effect_state.stop(block_id)
                    self._effect_state.record_packet_debug(f"FFB Op: STOP block={block_id}")
            except Exception:
                pass
            return

        # 0x0B: PID Block Free Report — remove an effect.
        if pkt_type == 0x0B:
            block_id = self._get_ebi(fns, data_ptr)
            if block_id is not None:
                self._effect_state.remove(block_id)
                self._effect_state.record_packet_debug(f"FFB Free: block={block_id}")
            return

        # 0x0C: PID Device Control Report.
        if pkt_type == 0x0C and fns.Ffb_h_DevCtrl is not None:
            ctrl = ctypes.c_uint(0)
            try:
                fns.Ffb_h_DevCtrl(data_ptr, ctypes.byref(ctrl))
                ctrl_val = int(ctrl.value)
                if ctrl_val == 1:  # Enable Actuators
                    pass
                elif ctrl_val == 2:  # Disable Actuators
                    self._effect_state.stop_all()
                elif ctrl_val == 3:  # Stop All Effects
                    self._effect_state.stop_all()
                elif ctrl_val == 4:  # Device Reset
                    self._effect_state.reset()
                elif ctrl_val == 5:  # Device Pause
                    self._effect_state.stop_all()
                elif ctrl_val == 6:  # Device Continue
                    for s in self._effect_state.slots.values():
                        s.playing = True

                self._effect_state.record_packet_debug(f"FFB DeviceCtrl: value={ctrl_val}")
            except Exception:
                pass
            return

    def _get_ebi(self, fns: _VJoyFns, data_ptr: ctypes.c_void_p) -> int | None:
        """Extract Effect Block Index from a packet."""
        if fns.Ffb_h_EBI is not None:
            ebi = ctypes.c_int(0)
            try:
                fns.Ffb_h_EBI(data_ptr, ctypes.byref(ebi))
                return int(ebi.value)
            except Exception:
                pass
        return None

    def poll_ffb(self) -> FfbCommand | None:
        # Callers should prefer effect_state.compute_force() directly.
        return None

    @property
    def effect_state(self) -> EffectState:
        return self._effect_state
