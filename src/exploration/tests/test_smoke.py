"""小测试：均匀划分与权重聚合不抛错。"""

from __future__ import annotations

import sys
from pathlib import Path

_SRC_ROOT = Path(__file__).resolve().parents[2]
_EXPL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SRC_ROOT))

from exploration.aggregation import aggregate_rect_weights, finalize_zone_weights  # noqa: E402
from exploration.config import AggregationParams  # noqa: E402
from exploration.detection_zones import load_detection_zones  # noqa: E402
from exploration.pipeline import workspace_from_zones  # noqa: E402


def test_uniform_grid_count():
    from exploration.zoning import build_uniform_grid

    cells = build_uniform_grid((0, 0, 10, 10), cell_size_m=2.5)
    assert len(cells) == 4 * 4


def test_pipeline_smoke():
    from exploration.zoning import build_uniform_grid

    zpath = _EXPL_ROOT / "data" / "zones.example.json"
    zones = load_detection_zones(zpath)
    ws = workspace_from_zones(zones, margin_m=0.2)
    agg = finalize_zone_weights({"A": 2, "B": 1}, {"A": 0.5, "B": 0.5}, AggregationParams())
    rw = aggregate_rect_weights(
        build_uniform_grid(ws, cell_size_m=1.2),
        zones,
        agg,
    )
    assert len(rw) >= 1


def test_random_start_reproducible():
    import random

    from exploration.planner import sample_random_start_cell_key
    from exploration.zoning import build_uniform_grid

    ws = (0.0, 0.0, 4.0, 4.0)
    cells = build_uniform_grid(ws, cell_size_m=1.0)
    rw = {(c.row, c.col): 0.1 * (c.row + c.col) for c in cells}
    r1 = random.Random(7)
    r2 = random.Random(7)
    a = sample_random_start_cell_key(cells, rw, ws, rng=r1)
    b = sample_random_start_cell_key(cells, rw, ws, rng=r2)
    assert a == b
