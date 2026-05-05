"""region_planner 行为 smoke + tiebreak 测试."""
from __future__ import annotations

import numpy as np

from coverage_planner.map_io import preprocess
from coverage_planner.planner import PlannerConfig
from coverage_planner.region_planner import (
    classify_region,
    config_for_kind,
    plan_partitions,
)


def _square_free(h: int, w: int) -> np.ndarray:
    f = np.ones((h, w), dtype=bool)
    f[0, :] = False
    f[-1, :] = False
    f[:, 0] = False
    f[:, -1] = False
    return f


def test_classify_open_square():
    free = _square_free(40, 40)
    g = preprocess(free)
    s = classify_region(g.free, g.edt, r_px=10.0, region_id=1)
    assert s.kind == "open"
    assert s.rectangularity > 0.85


def test_classify_narrow_corridor():
    free = np.zeros((40, 80), dtype=bool)
    free[18:22, 2:78] = True
    g = preprocess(free)
    s = classify_region(g.free, g.edt, r_px=10.0, region_id=2)
    assert s.kind == "narrow"
    assert s.width_ratio < 0.6


def test_config_for_kind_open_disables_medial_reflex():
    base = PlannerConfig(r=10.0, random_extra=500)
    cfg = config_for_kind(base, "open")
    assert cfg.use_hex and not cfg.use_medial and not cfg.use_reflex
    assert cfg.random_extra <= 50


def test_plan_partitions_smoke_two_regions():
    h, w = 60, 120
    free = np.zeros((h, w), dtype=bool)
    free[5:55, 5:55] = True
    free[5:55, 65:115] = True
    free[28:32, 55:65] = True
    labels = np.zeros((h, w), dtype=np.int32)
    labels[:, :60] = 1
    labels[:, 60:] = 2

    res = plan_partitions(
        free_full=free,
        labels=labels,
        r_px=8.0,
        base_config=PlannerConfig(
            r=8.0, coverage_ratio=0.9, target_subsample=2,
            random_extra=100, enable_ils=False, verbose=False,
        ),
        verbose=False,
    )
    assert len(res.region_plans) == 2
    assert res.total_circles >= 2
    pts = res.all_points()
    assert pts.shape[0] == res.total_circles
