from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Calibration:
    """Persisted calibration data.

    center_motor_turns: encoder position (turns) that corresponds to steering center.
    """

    center_motor_turns: float


def load_calibration(path: str) -> Calibration | None:
    p = Path(path)
    if not p.exists():
        return None
    data = json.loads(p.read_text(encoding="utf-8"))
    try:
        center = float(data["center_motor_turns"])
    except Exception:
        return None
    return Calibration(center_motor_turns=center)


def save_calibration(path: str, calib: Calibration) -> None:
    p = Path(path)
    payload = {"center_motor_turns": float(calib.center_motor_turns)}
    p.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
