"""从 ROS map YAML/PGM 构建 ``nav_msgs/OccupancyGrid``（依赖 coverage_planner.map_io）."""
from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import OccupancyGrid

from map_tools.bootstrap import ensure_coverage_planner

ensure_coverage_planner()

from coverage_planner.map_io import (  # noqa: E402
    load_ros_map,
    load_ros_map_occupancy,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    ha = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(ha)
    q.w = math.cos(ha)
    return q


def build_occupancy_grid_msg(
    yaml_path: str | Path,
    *,
    frame_id: str = "map",
) -> OccupancyGrid:
    """读取磁盘地图，构造 ``OccupancyGrid``（原始三态：-1/0/100）."""
    yaml_path = Path(yaml_path)
    occ_hw, info = load_ros_map_occupancy(yaml_path)
    h, w = occ_hw.shape

    pose = Pose()
    pose.position.x = info.origin[0]
    pose.position.y = info.origin[1]
    pose.position.z = 0.0
    pose.orientation = yaw_to_quaternion(float(info.origin[2]))

    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.info.resolution = float(info.resolution)
    msg.info.width = int(w)
    msg.info.height = int(h)
    msg.info.origin = pose
    msg.data = occ_hw.astype("int8").ravel(order="C").tolist()
    return msg


def load_free_mask_for_planning(yaml_path: str | Path, **kwargs):
    """封装 :func:`coverage_planner.map_io.load_ros_map`，供规划脚本调用."""
    return load_ros_map(yaml_path, **kwargs)
