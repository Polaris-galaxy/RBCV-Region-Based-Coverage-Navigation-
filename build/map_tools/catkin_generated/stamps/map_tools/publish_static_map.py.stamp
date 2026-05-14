#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1：从磁盘 YAML+PGM 发布 ``nav_msgs/OccupancyGrid``（latched）。

参数 ``~map_yaml``：地图 yaml 路径；开发时可用 ``$(find map_tools)/maps/map7.yaml``。

依赖：同工作空间的 ``coverage_planner``（由 ``map_tools`` 自动加入 ``sys.path``）。
"""
from __future__ import annotations

import rospy
from nav_msgs.msg import OccupancyGrid

from map_tools.bridge import build_occupancy_grid_msg


def main() -> None:
    rospy.init_node("static_map_publisher", anonymous=False)
    yaml_path = rospy.get_param("~map_yaml", "")
    if not yaml_path:
        rospy.logfatal("请设置私有参数 ~map_yaml（例如 $(find map_tools)/maps/map7.yaml）")
        raise SystemExit(1)
    topic = rospy.get_param("~topic", "/map")
    frame_id = rospy.get_param("~frame_id", "map")
    rate_hz = float(rospy.get_param("~rate_hz", 1.0))

    pub = rospy.Publisher(topic, OccupancyGrid, queue_size=1, latch=True)
    rospy.loginfo("static_map_publisher: yaml=%s -> %s", yaml_path, topic)

    msg_template = build_occupancy_grid_msg(yaml_path, frame_id=frame_id)

    if rate_hz <= 1e-9:
        msg_template.header.stamp = rospy.Time.now()
        msg_template.info.map_load_time = rospy.Time.now()
        pub.publish(msg_template)
        rospy.spin()
        return

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        msg_template.header.stamp = rospy.Time.now()
        msg_template.info.map_load_time = msg_template.header.stamp
        pub.publish(msg_template)
        rate.sleep()


if __name__ == "__main__":
    main()
