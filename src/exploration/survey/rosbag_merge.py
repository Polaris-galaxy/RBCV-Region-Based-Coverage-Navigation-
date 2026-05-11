"""视觉检测结果 JSONL + ROS1 bag 位姿 → :class:`ObjectObservation` 列表。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from ..config import ObjectObservation
from ..rosbag_ros1 import iter_poses_ros1_topic
from .coverage_geometry import match_zone_for_robot
from .pose_timeline import PoseTimeline
from .zone_catalog import SurveyZone


@dataclass
class DetectionRecord:
    t_s: float
    class_name: str
    x: float
    y: float
    confidence: float = 1.0
    track_id: int | None = None


def load_detections_jsonl(path: str | Path) -> list[DetectionRecord]:
    path = Path(path)
    rows: list[DetectionRecord] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        d = json.loads(line)
        rows.append(
            DetectionRecord(
                t_s=float(d["t_s"]),
                class_name=str(d.get("class_name", d.get("class", "object"))),
                x=float(d["x"]),
                y=float(d["y"]),
                confidence=float(d.get("confidence", 1.0)),
                track_id=int(d["track_id"]) if d.get("track_id") is not None else None,
            )
        )
    rows.sort(key=lambda r: r.t_s)
    return rows


def poses_from_rosbag(path: str | Path, topic: str) -> PoseTimeline:
    samples = list(iter_poses_ros1_topic(path, topic=topic, on_unsupported_msgtype="skip"))
    return PoseTimeline.from_samples(samples)


def observations_from_bag_and_jsonl(
    *,
    bag_path: str | Path,
    odom_topic: str,
    catalog: list[SurveyZone],
    detections_jsonl: str | Path,
    max_time_delta_s: float = 0.25,
) -> tuple[list[ObjectObservation], PoseTimeline]:
    """对每条视觉记录，用 rosbag 插值得到机器人位置；仅当机器人处于某框**覆盖区**内时记入该框。

    物体世界坐标用检测算法给出的 ``(x,y)``；去重由后续 :func:`fusion.count_unique_objects_in_zones` 完成
    （track_id 优先，否则 DBSCAN）。
    """
    timeline = poses_from_rosbag(bag_path, odom_topic)
    dets = load_detections_jsonl(detections_jsonl)
    obs: list[ObjectObservation] = []
    if not timeline.t:
        return obs, timeline
    for d in dets:
        if d.t_s < timeline.t[0] - max_time_delta_s or d.t_s > timeline.t[-1] + max_time_delta_s:
            continue
        rx, ry = timeline.xy_at(d.t_s)
        hit = match_zone_for_robot(rx, ry, catalog)
        if hit is None:
            continue
        obs.append(
            ObjectObservation(
                timestamp_s=d.t_s,
                zone_id=hit.zone_id,
                class_name=d.class_name,
                x=d.x,
                y=d.y,
                confidence=d.confidence,
                track_id=d.track_id,
            )
        )
    return obs, timeline
