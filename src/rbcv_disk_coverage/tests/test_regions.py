"""region_io 栅格化与掩码逻辑单元测试."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from coverage_planner.map_io import RosMapInfo
from coverage_planner.region_io import (
    RegionRect,
    RegionsSpec,
    load_regions_json,
    mask_free_with_regions,
    rasterize_regions,
    split_labels_by_region,
)


def test_region_rect_sorts_bounds():
    r = RegionRect("x", xmax=1.0, xmin=3.0, ymax=2.0, ymin=0.0)
    assert r.xmin == 1.0 and r.xmax == 3.0 and r.ymin == 0.0 and r.ymax == 2.0


def test_rasterize_first_wins_overlap():
    info = RosMapInfo(
        image_path=Path("dummy.pgm"),
        resolution=1.0,
        origin=(0.0, 0.0, 0.0),
    )
    spec = RegionsSpec(
        frame_id="map",
        regions=[
            RegionRect("a", 0.0, 0.0, 2.0, 2.0),
            RegionRect("b", 1.0, 1.0, 3.0, 3.0),
        ],
    )
    labels = rasterize_regions(spec, info, shape=(4, 4))
    # center (0.5,0.5)->1 (1.5,0.5)->1 ...
    assert labels[0, 0] == 1
    overlap = labels[1, 1]
    assert overlap == 1


def test_mask_free(tmp_path: Path):
    free = np.ones((3, 3), dtype=bool)
    labels = np.array([[1, 1, 0], [1, 0, 0], [0, 0, 0]], dtype=np.int32)
    mf = mask_free_with_regions(free, labels)
    assert mf.sum() == 3


def test_split_labels():
    labels = np.array([[1, 2], [2, 0]], dtype=np.int32)
    d = split_labels_by_region(labels)
    assert set(d.keys()) == {1, 2}
    assert d[1].sum() == 1 and d[2].sum() == 2


def test_load_json_roundtrip(tmp_path: Path):
    p = tmp_path / "r.json"
    spec = RegionsSpec(
        frame_id="m",
        regions=[RegionRect("z", -1.0, -2.0, 0.5, 1.0)],
    )
    p.write_text(
        json.dumps(
            {
                "frame_id": spec.frame_id,
                "regions": [
                    {
                        "id": r.id,
                        "xmin": r.xmin,
                        "ymin": r.ymin,
                        "xmax": r.xmax,
                        "ymax": r.ymax,
                    }
                    for r in spec.regions
                ],
            }
        ),
        encoding="utf-8",
    )
    got = load_regions_json(p)
    assert got.frame_id == "m" and len(got.regions) == 1
    assert got.regions[0].id == "z"
