"""load_ros_map_occupancy 与示例地图一致性."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from coverage_planner.map_io import load_ros_map_occupancy


def test_load_ros_map_occupancy_map7():
    src_dir = Path(__file__).resolve().parents[2]
    yaml_path = src_dir / "map_tools" / "maps" / "map7.yaml"
    assert yaml_path.is_file(), f"missing fixture {yaml_path}"
    occ, info = load_ros_map_occupancy(yaml_path)
    assert occ.dtype == np.int8
    assert occ.shape[0] > 10 and occ.shape[1] > 10
    u = set(np.unique(occ).tolist())
    assert u.issubset({-1, 0, 100})
    assert info.resolution > 0
