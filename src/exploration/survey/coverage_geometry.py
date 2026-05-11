"""判定机器人位姿是否处于「可对某框做覆盖观测」的区域。"""

from __future__ import annotations

import math

from shapely.geometry import Point, Polygon
from shapely.geometry import box as shapely_box

from ..config import DetectionZone
from .zone_catalog import CoverageSpec, SurveyZone


def _footprint_poly(z: DetectionZone) -> Polygon:
    if z.polygon_xy:
        return Polygon(z.polygon_xy)
    assert z.bbox_xyxy is not None
    x0, y0, x1, y1 = z.bbox_xyxy
    return shapely_box(x0, y0, x1, y1)


def robot_in_zone_footprint(robot_x: float, robot_y: float, z: DetectionZone) -> bool:
    p = Point(robot_x, robot_y)
    poly = _footprint_poly(z)
    return bool(poly.covers(p) or poly.touches(p))


def robot_in_coverage_area(robot_x: float, robot_y: float, row: SurveyZone) -> bool:
    """若 ``coverage.sites_m`` 非空：机器人需在**检测框内**且距**至少一个覆盖圆心** ≤ ``radius_m``。

    若圆心列表为空：只要在检测框（多边形/矩形）内即视为处于该框的覆盖任务区
    （适用于尚未从 RBCV 导出圆心、暂以整框代替的情形）。
    """
    if not robot_in_zone_footprint(robot_x, robot_y, row.zone):
        return False
    cov = row.coverage
    if not cov.sites_m:
        return True
    r = max(cov.radius_m, 0.0)
    for sx, sy in cov.sites_m:
        if math.hypot(robot_x - sx, robot_y - sy) <= r + 1e-9:
            return True
    return False


def match_zone_for_robot(robot_x: float, robot_y: float, catalog: list[SurveyZone]) -> SurveyZone | None:
    """按 JSON 声明顺序，返回第一个匹配的检测区（重叠时先声明者优先）。"""
    for row in catalog:
        if robot_in_coverage_area(robot_x, robot_y, row):
            return row
    return None
