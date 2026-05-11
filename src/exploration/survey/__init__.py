"""Rosbag / 视觉对齐、覆盖几何与一键语义导航栈。"""

from .coverage_geometry import match_zone_for_robot, robot_in_coverage_area
from .pose_timeline import PoseTimeline
from .rosbag_merge import load_detections_jsonl, observations_from_bag_and_jsonl, poses_from_rosbag
from .stack import run_semantic_stack_from_files
from .zone_catalog import CoverageSpec, SurveyZone, load_survey_zones, survey_zones_as_detection_list

__all__ = [
    "CoverageSpec",
    "SurveyZone",
    "load_survey_zones",
    "survey_zones_as_detection_list",
    "PoseTimeline",
    "poses_from_rosbag",
    "load_detections_jsonl",
    "observations_from_bag_and_jsonl",
    "robot_in_coverage_area",
    "match_zone_for_robot",
    "run_semantic_stack_from_files",
]
