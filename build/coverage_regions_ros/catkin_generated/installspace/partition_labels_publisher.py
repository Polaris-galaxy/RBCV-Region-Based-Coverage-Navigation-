#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 Noetic：订阅一张 OccupancyGrid(/map)，按 JSON 长方形分区栅格化并发布分区标签栅格。

发布的 OccupancyGrid 与输入地图同分辨率、同尺寸、同坐标约定；
data[k] 语义约定（非经典占用栅格）::
    -1 : 不属于任一长方形分区（可与 /map 自由空间按位与）
    1..127 : 分区序号（与 JSON regions 数组顺序一致，从 1 起）

JSON Schema（与 coverage_planner.region_io 一致）::
    {"frame_id": "map", "regions": [{"id":"A","xmin":..,"ymin":..,"xmax":..,"ymax":..}, ...]}

参数（私有命名空间 ~）::
    ~regions_json (str, 必填)   分区 JSON 路径
    ~map_topic (str)           默认 /map
    ~partition_topic (str)     默认 /region_partition_labels
    ~publish_markers (bool)    是否在 RViz 画长方形边线，默认 true
    ~marker_topic (str)       默认 /region_partition_markers
    ~rate_hz (float)          定时发布频率；<=0 则仅在收到 /map 后 latch 发布一次

TODO: 包名 / maintainer / email 可按工程改名。
"""
from __future__ import annotations

import json
import math
import threading

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    from tf.transformations import euler_from_quaternion
except ImportError:
    euler_from_quaternion = None


def _load_regions(path: str) -> tuple[str, list[dict]]:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    frame = str(raw.get("frame_id", "map"))
    regs = raw["regions"]
    out = []
    for it in regs:
        xa, xb = sorted((float(it["xmin"]), float(it["xmax"])))
        ya, yb = sorted((float(it["ymin"]), float(it["ymax"])))
        out.append({"id": str(it.get("id", "")), "xmin": xa, "xmax": xb, "ymin": ya, "ymax": yb})
    return frame, out


def _yaw_from_map_origin(ori) -> float:
    if euler_from_quaternion is None:
        return 0.0
    q = (ori.x, ori.y, ori.z, ori.w)
    roll, pitch, yaw = euler_from_quaternion(q)
    return float(yaw)


def _cell_center_in_map_frame(m: OccupancyGrid, row: int, col: int) -> tuple[float, float]:
    """返回第 (row,col) 格中心在 map 坐标系下的 (mx, my)，米."""
    ox = m.info.origin.position.x
    oy = m.info.origin.position.y
    res = float(m.info.resolution)
    gx = (col + 0.5) * res
    gy = (row + 0.5) * res
    yaw = _yaw_from_map_origin(m.info.origin.orientation)
    c, s = math.cos(yaw), math.sin(yaw)
    mx = ox + gx * c - gy * s
    my = oy + gx * s + gy * c
    return mx, my


def _rasterize(m: OccupancyGrid, regions: list[dict]) -> list:
    h, w = int(m.info.height), int(m.info.width)
    n = h * w
    data = [-1] * n

    def idx(row: int, col: int) -> int:
        return row * w + col

    for ri, reg in enumerate(regions, start=1):
        tag = min(ri, 127)
        for row in range(h):
            for col in range(w):
                ii = idx(row, col)
                if data[ii] != -1:
                    continue
                mx, my = _cell_center_in_map_frame(m, row, col)
                if (
                    reg["xmin"] <= mx <= reg["xmax"]
                    and reg["ymin"] <= my <= reg["ymax"]
                ):
                    data[ii] = tag
    return [int(v) for v in data]


def _markers_for_regions(m: OccupancyGrid, regions: list[dict]) -> MarkerArray:
    arr = MarkerArray()
    stamp = rospy.Time.now()
    res = float(m.info.resolution)

    for i, reg in enumerate(regions):
        mx0, mx1 = reg["xmin"], reg["xmax"]
        my0, my1 = reg["ymin"], reg["ymax"]
        corners_map = [
            (mx0, my0),
            (mx1, my0),
            (mx1, my1),
            (mx0, my1),
            (mx0, my0),
        ]
        pts = []
        for mx, my in corners_map:
            p = Point()
            p.x = mx
            p.y = my
            p.z = 0.05
            pts.append(p)

        mk = Marker()
        mk.header = m.header
        mk.ns = "region_rect"
        mk.id = i
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = max(res * 2.0, 0.02)
        mk.color = ColorRGBA(1.0, 0.3, 0.0, 0.95)
        mk.points = pts
        mk.lifetime = rospy.Duration(0)
        mk.header.stamp = stamp
        arr.markers.append(mk)
    return arr


class PartitionPublisher:
    def __init__(self) -> None:
        rospy.init_node("partition_labels_publisher", anonymous=False)

        regions_json = rospy.get_param("~regions_json", "")
        if not regions_json:
            rospy.logfatal("参数 ~regions_json 未设置（长方形分区 JSON 路径）")
            raise SystemExit(1)

        self._regions_frame, self._regions = _load_regions(regions_json)
        rospy.loginfo("已载入分区 %d 个，JSON frame_id=%s", len(self._regions), self._regions_frame)

        self._map_topic = rospy.get_param("~map_topic", "/map")
        self._partition_topic = rospy.get_param("~partition_topic", "/region_partition_labels")
        self._publish_markers = rospy.get_param("~publish_markers", True)
        self._marker_topic = rospy.get_param("~marker_topic", "/region_partition_markers")
        rate_hz = float(rospy.get_param("~rate_hz", 1.0))

        self._latest_map: OccupancyGrid | None = None
        self._lock = threading.Lock()

        self._pub_part = rospy.Publisher(self._partition_topic, OccupancyGrid, queue_size=1, latch=True)
        self._pub_mk = rospy.Publisher(self._marker_topic, MarkerArray, queue_size=1, latch=True)

        rospy.Subscriber(self._map_topic, OccupancyGrid, self._on_map)

        if rate_hz > 1e-6:
            rospy.Timer(rospy.Duration(1.0 / rate_hz), self._on_timer)
        else:
            rospy.loginfo("rate_hz<=0：仅在收到地图后发布一次（latch）")

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._latest_map = msg
        self._publish_once(msg)

    def _on_timer(self, _evt) -> None:
        with self._lock:
            m = self._latest_map
        if m is None:
            return
        self._publish_once(m)

    def _publish_once(self, m: OccupancyGrid) -> None:
        labels = _rasterize(m, self._regions)
        out = OccupancyGrid()
        out.header = m.header
        out.info = m.info
        out.data = labels

        if self._regions_frame and self._regions_frame != m.header.frame_id:
            rospy.logwarn_throttle(
                30.0,
                "JSON frame_id=%s 与地图 header.frame_id=%s 不一致，仍以地图帧发布",
                self._regions_frame,
                m.header.frame_id,
            )

        self._pub_part.publish(out)
        rospy.logdebug("发布分区栅格 -> %s (%dx%d)", self._partition_topic, m.info.width, m.info.height)

        if self._publish_markers:
            self._pub_mk.publish(_markers_for_regions(m, self._regions))


def main() -> None:
    PartitionPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
