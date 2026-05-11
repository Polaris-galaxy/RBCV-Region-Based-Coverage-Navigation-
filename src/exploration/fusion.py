"""物体记录：连续位姿下合并同一物体、得到每个检测框内唯一实例数量。

策略（默认）：
    1. 观测按 (zone_id, class_name) 分组；
    2. 组内对平面坐标 (x,y) 做 DBSCAN，eps=merge_radius_m，min_samples=1；
    3. 簇数 = 该框内该类 **唯一物体数**。

若视觉已提供稳定的 ``track_id``，则优先按 track_id 去重。
"""

from __future__ import annotations

import math
from collections import defaultdict

import numpy as np

from .config import ObjectObservation


def _unique_with_tracks(obs: list[ObjectObservation]) -> int:
    ids = {o.track_id for o in obs if o.track_id is not None}
    if ids and len(ids) == len([o for o in obs if o.track_id is not None]):
        return len(ids)
    return -1


def count_unique_objects_in_zones(
    observations: list[ObjectObservation],
    *,
    merge_radius_m: float = 0.45,
) -> dict[str, dict[str, int]]:
    """返回 ``{zone_id: {class_name: unique_count}}``。"""
    by_zone_class: dict[tuple[str, str], list[ObjectObservation]] = defaultdict(list)
    for o in observations:
        by_zone_class[(o.zone_id, o.class_name)].append(o)

    out: dict[str, dict[str, int]] = defaultdict(dict)
    for (zid, cname), group in by_zone_class.items():
        ut = _unique_with_tracks(group)
        if ut >= 0:
            out[zid][cname] = ut
            continue
        if len(group) == 1:
            out[zid][cname] = 1
            continue
        try:
            from sklearn.cluster import DBSCAN  # type: ignore
        except ImportError:
            # 无 sklearn 时退化为栅格分桶
            gx = [round(o.x / merge_radius_m) for o in group]
            gy = [round(o.y / merge_radius_m) for o in group]
            cells = set(zip(gx, gy))
            out[zid][cname] = len(cells)
            continue

        X = np.array([[o.x, o.y] for o in group], dtype=np.float64)
        labels = DBSCAN(eps=merge_radius_m, min_samples=1).fit_predict(X)
        n_clu = len({int(l) for l in labels if int(l) >= 0})
        out[zid][cname] = max(1, n_clu)

    return {k: dict(v) for k, v in out.items()}


def zone_total_object_count(per_zone_class: dict[str, dict[str, int]]) -> dict[str, int]:
    return {z: sum(d.values()) for z, d in per_zone_class.items()}


def assign_robot_zone(
    x: float,
    y: float,
    zone_polygons: dict[str, list[tuple[float, float]]],
) -> str | None:
    """点在多边形内则属于该 zone（首个命中）。需要更严谨可改绕序."""
    try:
        from shapely.geometry import Point, Polygon
    except ImportError:
        raise ImportError("需要 shapely: pip install shapely")

    p = Point(x, y)
    for zid, poly in zone_polygons.items():
        if Polygon(poly).covers(p):
            return zid
    return None
