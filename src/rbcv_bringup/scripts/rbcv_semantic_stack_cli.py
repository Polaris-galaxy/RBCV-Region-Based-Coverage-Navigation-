#!/usr/bin/env python3
"""阶段二：转发到 ``run_semantic_stack``。

- 命令行：与 ``exploration/scripts/run_semantic_stack.py`` 相同。
- 经 roslaunch：在节点上设置 ``RBCV_SEMANTIC_FROM_LAUNCH=1`` 及 ``RBCV_*`` 环境变量（见 ``rbcv_stage2_semantic_explore.launch``），由本脚本组装 ``sys.argv``。
"""
from __future__ import annotations

import os
import sys
from pathlib import Path


def _ensure_src_path() -> Path:
    src = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(src))
    return src


def main() -> None:
    _ensure_src_path()
    from exploration.scripts.run_semantic_stack import main as stack_main

    if os.environ.get("RBCV_SEMANTIC_FROM_LAUNCH") == "1":
        survey = os.environ.get("RBCV_SURVEY", "").strip()
        out = os.environ.get("RBCV_OUT", "").strip()
        if not survey or not out:
            print("错误：launch 需设置 RBCV_SURVEY 与 RBCV_OUT。", file=sys.stderr)
            sys.exit(2)
        argv = [sys.argv[0], "--survey", survey, "--out", out]
        bag = os.environ.get("RBCV_BAG", "").strip()
        if bag:
            topic = os.environ.get("RBCV_TOPIC", "/odom").strip() or "/odom"
            det = os.environ.get("RBCV_DET", "").strip()
            if not det:
                print("错误：已设置 RBCV_BAG 时必须同时设置 RBCV_DET。", file=sys.stderr)
                sys.exit(2)
            argv += ["--bag", bag, "--topic", topic, "--detections", det]
        cs = os.environ.get("RBCV_CELL", "").strip()
        if cs:
            try:
                csf = float(cs)
            except ValueError:
                csf = 0.0
            if csf > 0:
                argv += ["--cell-size", str(csf)]
        sys.argv = argv
    else:
        argv = [sys.argv[0]]
        for a in sys.argv[1:]:
            if a.startswith("__") or ":=" in a:
                continue
            argv.append(a)
        sys.argv = argv

    stack_main()


if __name__ == "__main__":
    main()
