"""探索式加权覆盖：检测框、物体记录、矩形分区、代价地图与路径."""

from .config import (
    AggregationParams,
    DetectionZone,
    InitialRegionWeight,
    ObjectObservation,
    PathPlannerParams,
    SpatialRectPrior,
    StartSamplingParams,
)
from .detection_zones import load_detection_zones, save_detection_zones
from .zoning import build_uniform_grid
from .aggregation import aggregate_rect_weights, finalize_zone_weights
from .costmap import weights_to_traversal_cost, build_grid_cost_fields
from .planner import plan_exploration_tour, sample_random_start_cell_key
from .regions_bridge import load_zones_from_regions_json
from .survey.stack import run_semantic_stack_from_files

__all__ = [
    "AggregationParams",
    "DetectionZone",
    "InitialRegionWeight",
    "ObjectObservation",
    "PathPlannerParams",
    "SpatialRectPrior",
    "StartSamplingParams",
    "load_detection_zones",
    "save_detection_zones",
    "build_uniform_grid",
    "aggregate_rect_weights",
    "finalize_zone_weights",
    "weights_to_traversal_cost",
    "build_grid_cost_fields",
    "plan_exploration_tour",
    "sample_random_start_cell_key",
    "load_zones_from_regions_json",
    "run_semantic_stack_from_files",
]
