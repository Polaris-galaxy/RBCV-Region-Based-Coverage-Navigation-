"""确保可导入 ``coverage_planner``（源码树 / catkin devel / 环境变量）."""
from __future__ import annotations

import os
import sys
from pathlib import Path


def ensure_coverage_planner() -> None:
    try:
        import coverage_planner  # noqa: F401

        return
    except ImportError:
        pass

    env = os.environ.get("COVERAGE_PLANNER_SRC")
    if env:
        p = Path(env)
        if (p / "coverage_planner" / "map_io.py").is_file():
            sys.path.insert(0, str(p))
            return

    here = Path(__file__).resolve()
    for anc in here.parents:
        cand = anc / "coverage_planner"
        if (cand / "coverage_planner" / "map_io.py").is_file():
            sys.path.insert(0, str(cand))
            return
        cand_src = anc / "src" / "coverage_planner"
        if (cand_src / "coverage_planner" / "map_io.py").is_file():
            sys.path.insert(0, str(cand_src))
            return
