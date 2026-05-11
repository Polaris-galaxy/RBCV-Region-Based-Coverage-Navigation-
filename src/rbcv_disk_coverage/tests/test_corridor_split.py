"""corridor_split 几何：横纵标签 + 划条数量."""
from __future__ import annotations

import numpy as np

from coverage_planner.corridor_split import (
    corridor_component_axes,
    corridor_orientation_label,
    wide_corridor_strip_centers,
)


def test_orientation_horizontal_dominates_x():
    comp = np.stack(
        [np.ones(80) * 10, np.arange(80)],
        axis=1,
    ).astype(np.float64)
    main, _orth = corridor_component_axes(comp)
    assert corridor_orientation_label(main) == "horizontal"


def test_orientation_vertical_dominates_y():
    comp = np.stack([np.arange(60), np.ones(60) * 20], axis=1).astype(np.float64)
    main, _orth = corridor_component_axes(comp)
    assert corridor_orientation_label(main) == "vertical"


def test_narrow_corridor_returns_zero_strip_lines():
    free = np.zeros((40, 200), dtype=bool)
    free[17:23, 2:198] = True
    edt = np.zeros(free.shape, dtype=np.float32)
    ys, xs = np.where(free)
    edt[ys, xs] = np.minimum(ys - 17 + 1, 22 - ys + 1).astype(np.float32)
    comp = np.stack([ys.astype(float), xs.astype(float)], axis=1)
    pts, n_lines, orient = wide_corridor_strip_centers(
        free, edt, comp, r_px=10.0, resolution_m_per_px=0.05,
        strip_width_m=4.0, alpha=0.9,
    )
    assert n_lines == 0
    assert pts.shape[0] == 0
    assert orient == "narrow"


def test_wide_enough_registers_strip_lines():
    """人工 EDT：验证 n=ceil(W/strip_w)（默认对照 4m）。"""
    res = 0.05
    h, w = 100, 140
    free = np.ones((h, w), dtype=bool)
    edt = np.full((h, w), 55.0, dtype=np.float32)  # 2*55px * res = 5.5 m
    comp = np.stack(
        [np.ones(100) * 50.0, np.linspace(12.0, 127.0, 100)],
        axis=1,
    ).astype(np.float64)
    strip_w = 4.0
    _pts, n_lines, orient = wide_corridor_strip_centers(
        free, edt, comp, r_px=12.0, resolution_m_per_px=res,
        strip_width_m=strip_w, alpha=0.9,
    )
    width_px = 2.0 * 55.0
    width_m = width_px * res
    assert abs(width_m - 5.5) < 1e-6
    assert n_lines == int(np.ceil(width_m / strip_w)) == 2
    assert orient in ("horizontal", "vertical")
