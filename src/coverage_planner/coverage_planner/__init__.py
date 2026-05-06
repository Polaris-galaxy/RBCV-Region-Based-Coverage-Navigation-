"""coverage_planner: 含障碍大地图等半径圆盘覆盖规划器。

为了避免在仅使用部分子模块时强制依赖 ``scikit-image`` / ``imageio`` / ``matplotlib``,
本包采用 PEP 562 的 ``__getattr__`` 实现 **按需懒加载**。
"""
from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "GridMap",
    "load_map",
    "load_ros_map",
    "load_ros_map_occupancy",
    "RosMapInfo",
    "RegionRect",
    "RegionsSpec",
    "load_regions_json",
    "save_regions_json",
    "rasterize_regions",
    "union_region_mask",
    "mask_free_with_regions",
    "split_labels_by_region",
    "meters_to_pixels",
    "clean_map",
    "preprocess",
    "generate_candidates",
    "build_coverage",
    "visible_disk",
    "augment_for_feasibility",
    "greedy_set_cover",
    "find_unreachable_targets",
    "ils",
    "SolutionState",
    "plan_coverage",
    "PlannerConfig",
    "PlannerResult",
    "RegionShape",
    "RegionPlan",
    "PartitionResult",
    "classify_region",
    "config_for_kind",
    "plan_partitions",
    "globally_trim_solution",
]

_LAZY: dict[str, str] = {
    "GridMap": "coverage_planner.map_io",
    "load_map": "coverage_planner.map_io",
    "load_ros_map": "coverage_planner.map_io",
    "load_ros_map_occupancy": "coverage_planner.map_io",
    "RosMapInfo": "coverage_planner.map_io",
    "RegionRect": "coverage_planner.region_io",
    "RegionsSpec": "coverage_planner.region_io",
    "load_regions_json": "coverage_planner.region_io",
    "save_regions_json": "coverage_planner.region_io",
    "rasterize_regions": "coverage_planner.region_io",
    "union_region_mask": "coverage_planner.region_io",
    "mask_free_with_regions": "coverage_planner.region_io",
    "split_labels_by_region": "coverage_planner.region_io",
    "meters_to_pixels": "coverage_planner.map_io",
    "clean_map": "coverage_planner.map_io",
    "preprocess": "coverage_planner.map_io",
    "generate_candidates": "coverage_planner.candidates",
    "build_coverage": "coverage_planner.visibility",
    "visible_disk": "coverage_planner.visibility",
    "augment_for_feasibility": "coverage_planner.visibility",
    "greedy_set_cover": "coverage_planner.set_cover",
    "find_unreachable_targets": "coverage_planner.set_cover",
    "ils": "coverage_planner.refine",
    "SolutionState": "coverage_planner.refine",
    "plan_coverage": "coverage_planner.planner",
    "PlannerConfig": "coverage_planner.planner",
    "PlannerResult": "coverage_planner.planner",
    "RegionShape": "coverage_planner.region_planner",
    "RegionPlan": "coverage_planner.region_planner",
    "PartitionResult": "coverage_planner.region_planner",
    "classify_region": "coverage_planner.region_planner",
    "config_for_kind": "coverage_planner.region_planner",
    "plan_partitions": "coverage_planner.region_planner",
    "globally_trim_solution": "coverage_planner.region_planner",
}


def __getattr__(name: str) -> Any:
    if name in _LAZY:
        mod = import_module(_LAZY[name])
        value = getattr(mod, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module 'coverage_planner' has no attribute {name!r}")
