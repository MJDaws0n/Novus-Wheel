#!/usr/bin/env python3

from __future__ import annotations

import sys
from pathlib import Path


def _fix_sys_path() -> None:
    """Ensure the repo root is importable even when run from scripts/.

    When you run `python novus_wheel.py` from within `scripts/`, that folder is
    put on sys.path and contains `novus_wheel.py`, which would shadow the real
    package directory `../novus_wheel/`. We remove `scripts/` from sys.path and
    prepend the repo root.
    """

    scripts_dir = Path(__file__).resolve().parent
    repo_root = scripts_dir.parent

    # Drop entries that resolve to scripts_dir (including empty-string cwd).
    new_path: list[str] = []
    for p in sys.path:
        try:
            resolved = Path(p).resolve() if p else Path.cwd().resolve()
        except Exception:
            resolved = None
        if resolved is not None and resolved == scripts_dir:
            continue
        new_path.append(p)

    sys.path[:] = new_path
    sys.path.insert(0, str(repo_root))


_fix_sys_path()

from novus_wheel.app import main  # noqa: E402


if __name__ == "__main__":
    main()
