#!/usr/bin/env python3
"""地图 + 分区 JSON → 圆盘覆盖 → ``survey_zones.json``（检测框 bbox + coverage.sites_m）。

供 ``rbcv_stage1_partitions_to_survey.launch`` 调用；规划逻辑与 ``demo_regions`` 一致（环境变量相同）。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import rospy


def _import_demo_regions():
    """从任意 cwd 定位 ``rbcv_disk_coverage/scripts/demo_regions.py``。"""
    here = Path(__file__).resolve()
    for anc in here.parents:
        scripts = anc / "rbcv_disk_coverage" / "scripts" / "demo_regions.py"
        if scripts.is_file():
            rdc = anc / "rbcv_disk_coverage"
            sys.path.insert(0, str(rdc))
            sys.path.insert(0, str(rdc / "scripts"))
            import demo_regions  # type: ignore[import-untyped]

            return demo_regions
    raise RuntimeError("找不到 rbcv_disk_coverage/scripts/demo_regions.py，请保持 catkin src 目录结构。")


def main() -> None:
    pa = argparse.ArgumentParser(description="分区 JSON + 覆盖规划 → survey_zones.json")
    pa.add_argument("map_yaml", help="ROS 风格地图 YAML")
    pa.add_argument("regions_json", help="分区 regions.json")
    pa.add_argument("radius_m", type=float, help="覆盖半径（米）")
    pa.add_argument("survey_out", help="输出的 survey_zones.json 路径")
    argv = rospy.myargv(sys.argv)
    args = pa.parse_args(argv[1:])

    dr = _import_demo_regions()
    from coverage_planner.region_io import load_regions_json

    spec = load_regions_json(args.regions_json)
    partition, info, _free, _r_px, _n_rays_plan, _geo = dr.run_partition_demo(
        args.map_yaml,
        args.radius_m,
        args.regions_json,
    )
    payload = dr.survey_zones_dict_from_partition(
        spec, partition, info, args.radius_m
    )
    outp = Path(args.survey_out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    outp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[regions_to_survey_zones] 已写入 {outp}")


if __name__ == "__main__":
    main()
