#!/usr/bin/env python3
"""从 ROS1 bag 拉取平面轨迹（需先 inspect 选好 topic）。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_SRC_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SRC_ROOT))

from exploration.rosbag_ros1 import inspect_rosbag1, iter_poses_ros1_topic


def main() -> None:
    pa = argparse.ArgumentParser(description="解码 ROS1 bag 中某话题 -> (t,x,y,yaw)")
    pa.add_argument("bag", type=str)
    pa.add_argument("--topic", required=True)
    pa.add_argument("--limit", type=int, default=50, help="无 --out 时只打印前 N 条")
    pa.add_argument("--out", type=str, default="", help="JSON Lines 输出（写全 bag 该话题）")
    pa.add_argument("--strict", action="store_true")
    args = pa.parse_args()

    inspect_rosbag1(args.bag)

    mode = "raise" if args.strict else "skip"
    it = iter_poses_ros1_topic(args.bag, topic=args.topic, on_unsupported_msgtype=mode)

    if args.out:
        outp = Path(args.out)
        outp.parent.mkdir(parents=True, exist_ok=True)
        n = 0
        with outp.open("w", encoding="utf-8") as f:
            for tup in it:
                f.write(json.dumps(list(tup)) + "\n")
                n += 1
        print(f"已写 {n} 行 -> {outp}")
        return

    pts: list[list[float]] = []
    for i, tup in enumerate(it):
        pts.append(list(tup))
        if i + 1 >= args.limit:
            break
    payload = [{"t_s": a[0], "x": a[1], "y": a[2], "yaw": a[3]} for a in pts]
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
