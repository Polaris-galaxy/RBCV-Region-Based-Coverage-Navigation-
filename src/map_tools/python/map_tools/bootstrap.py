"""确保可导入 ``coverage_planner``（几何覆盖 Python 包；源码位于 ``rbcv_disk_coverage/``）."""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _try_insert_package_root(root: Path) -> bool:
    """``root`` 为含子目录 ``coverage_planner/``（内有 map_io.py）的那一层。"""
    inner = root / "coverage_planner" / "map_io.py"
    if inner.is_file():
        sys.path.insert(0, str(root.resolve()))
        return True
    return False


def ensure_coverage_planner() -> None:
    try:
        import coverage_planner  # noqa: F401

        return
    except ImportError:
        pass

    for key in ("RBCV_DISK_COVERAGE_SRC", "COVERAGE_PLANNER_SRC"):
        env = os.environ.get(key)
        if env and _try_insert_package_root(Path(env)):
            return

    here = Path(__file__).resolve()
    for anc in here.parents:
        if _try_insert_package_root(anc / "rbcv_disk_coverage"):
            return
        if _try_insert_package_root(anc / "coverage_planner"):
            return
        if _try_insert_package_root(anc / "src" / "rbcv_disk_coverage"):
            return
        if _try_insert_package_root(anc / "src" / "coverage_planner"):
            return
