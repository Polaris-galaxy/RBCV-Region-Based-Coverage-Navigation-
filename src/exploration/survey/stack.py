"""从 survey JSON +（可选）rosbag/检测文件 到闭合探索路径的一站式入口。"""

from __future__ import annotations

from pathlib import Path

from ..config import (
    AggregationParams,
    InitialRegionWeight,
    PathPlannerParams,
    SpatialRectPrior,
    StartSamplingParams,
)
from ..pipeline import run_post_survey_planning
from .rosbag_merge import observations_from_bag_and_jsonl
from .zone_catalog import load_survey_zones, survey_zones_as_detection_list, workspace_xyxy_from_survey


def run_semantic_stack_from_files(
    *,
    survey_zones_json: str | Path,
    workspace_xyxy: tuple[float, float, float, float] | None,
    priors: list[InitialRegionWeight],
    bag_path: str | Path | None = None,
    odom_topic: str | None = None,
    detections_jsonl: str | Path | None = None,
    spatial_rect_priors: list[SpatialRectPrior] | None = None,
    agg: AggregationParams | None = None,
    path_params: PathPlannerParams | None = None,
    cell_size_m: float | None = None,
    merge_radius_m: float = 0.45,
    start_cell_key: tuple[int, int] | None = None,
    start_sampling: StartSamplingParams | None = None,
) -> dict:
    """若提供 ``bag_path`` + ``odom_topic`` + ``detections_jsonl``，则先做时空对齐调查；否则观测列表为空（仅靠初始权重与先验）。"""
    catalog = load_survey_zones(survey_zones_json)
    zones = survey_zones_as_detection_list(catalog)
    ws = workspace_xyxy if workspace_xyxy is not None else workspace_xyxy_from_survey(catalog)
    observations: list = []
    n_pose_samples = 0
    if bag_path and odom_topic and detections_jsonl:
        observations, timeline = observations_from_bag_and_jsonl(
            bag_path=bag_path,
            odom_topic=odom_topic,
            catalog=catalog,
            detections_jsonl=detections_jsonl,
        )
        n_pose_samples = len(timeline.t)
    result = run_post_survey_planning(
        zones=zones,
        observations=observations,
        workspace_xyxy=ws,
        priors=priors,
        agg=agg,
        path_params=path_params,
        cell_size_m=cell_size_m,
        merge_radius_m=merge_radius_m,
        start_cell_key=start_cell_key,
        start_sampling=start_sampling,
        spatial_rect_priors=spatial_rect_priors,
    )
    result["survey_input"] = {
        "zones_json": str(Path(survey_zones_json).resolve()),
        "n_raw_observations": len(observations),
        "n_pose_samples": n_pose_samples,
    }
    return result
