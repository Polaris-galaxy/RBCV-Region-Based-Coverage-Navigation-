#!/usr/bin/env python3
"""ROS1：订阅 /rbcv/semantic_detections（std_msgs/String，单条 JSON），按行写入 JSONL。

与 RBCV ``exploration/data/detections.example.jsonl`` 格式兼容，可直接用于
``run_semantic_stack.py --detections``.

运行::

    rosrun <pkg> detection_recorder_node.py _output_path:=$(pwd)/my_detections.jsonl

参数：
    ~detections_topic  默认 /rbcv/semantic_detections
    ~output_path       输出文件（会追加写入）
"""

from __future__ import annotations

import json
import os
import sys


def main() -> None:
    try:
        import rospy
        from std_msgs.msg import String
    except ImportError as e:
        raise SystemExit("需要 rospy：请在 ROS1 Noetic 环境中运行。\n" + str(e)) from e

    rospy.init_node("rbcv_detection_recorder", anonymous=True)
    path = rospy.get_param("~output_path", "")
    topic = rospy.get_param("~detections_topic", "/rbcv/semantic_detections")
    if not path:
        rospy.logfatal("设置 ~output_path:=/路径/detections.jsonl")
        sys.exit(2)
    rospy.loginfo("录制 %s -> %s", topic, path)

    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)

    f = open(path, "a", encoding="utf-8")

    def on_msg(msg: String) -> None:
        line = (msg.data or "").strip()
        if not line:
            return
        try:
            d = json.loads(line)
            json.dumps(d)
        except Exception as e:
            rospy.logwarn("跳过非 JSON: %s (%s)", line[:120], e)
            return
        f.write(line + "\n")
        f.flush()

    rospy.Subscriber(topic, String, on_msg, queue_size=500)
    rospy.on_shutdown(f.close)
    rospy.spin()


if __name__ == "__main__":
    main()
