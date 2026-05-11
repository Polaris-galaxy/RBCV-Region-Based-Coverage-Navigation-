#!/usr/bin/env python3
"""ROS1 bag：列出话题与消息类型，便于你找到位姿 topic。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

_SRC_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SRC_ROOT))

from exploration.rosbag_ros1 import TopicInfo, inspect_rosbag1


def main() -> None:
    if len(sys.argv) < 2:
        print("用法: py scripts/inspect_rosbag1.py <xxx.bag> [--json]")
        sys.exit(1)
    bag = Path(sys.argv[1])
    rows: list[TopicInfo] = inspect_rosbag1(bag)
    as_dicts = [{"topic": r.topic, "msgtype": r.msgtype, "msgcount": r.msgcount} for r in rows]

    if "--json" in sys.argv:
        print(json.dumps({"bag": str(bag), "topics": as_dicts}, ensure_ascii=False, indent=2))
        return

    print(f"[rosbag] {bag} 话题数 = {len(rows)}")
    for r in rows:
        nc = "" if r.msgcount < 0 else f", 约{r.msgcount}条"
        print(f"  {r.topic:<48} type={r.msgtype}{nc}")
    print("\n候选位姿类常见 topic：含 odom、gimbal、pms、localization、robot_pose、eKF 关键词者；")
    print("对应类型常为 nav_msgs/Odometry 或 geometry_msgs/PoseStamped。")


if __name__ == "__main__":
    main()
