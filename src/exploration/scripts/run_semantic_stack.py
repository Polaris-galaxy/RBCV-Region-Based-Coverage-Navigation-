#!/usr/bin/env python3
"""语义导航栈：survey_zones.json + ROS1 bag 位姿 + 视觉 JSONL → 权重格 + 闭合探索路径。

视觉侧请将每条检测写为一行 JSON（UTF-8），字段示例见 ``exploration/data/detections.example.jsonl``。

在仓库 ``src/`` 下执行::

    py exploration/scripts/run_semantic_stack.py ^
      --survey exploration/data/survey_zones.example.json ^
      --bag D:\\record.bag --topic /odom ^
      --detections exploration/data/detections.example.jsonl ^
      --out exploration/data/out_semantic_plan.json

不加 ``--bag`` 时仅根据检测框初始权重与（可选）空观测跑通网格与路径，用于接线前自检。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SRC))

from exploration.config import (  # noqa: E402
    AggregationParams,
    InitialRegionWeight,
    PathPlannerParams,
    SpatialRectPrior,
)
from exploration.survey.stack import run_semantic_stack_from_files  # noqa: E402
from exploration.survey.zone_catalog import load_survey_zones, workspace_xyxy_from_survey  # noqa: E402


def _parse_priors(raw: str | None) -> list[InitialRegionWeight]:
    if not raw:
        return []
    data = json.loads(raw)
    out: list[InitialRegionWeight] = []
    for it in data:
        out.append(
            InitialRegionWeight(
                name=str(it.get("name", "prior")),
                weight=float(it["weight"]),
                applies_to_zone_ids=[str(x) for x in it["applies_to_zone_ids"]]
                if it.get("applies_to_zone_ids")
                else None,
            )
        )
    return out


def _parse_rect_priors(raw: str | None) -> list[SpatialRectPrior]:
    if not raw:
        return []
    data = json.loads(raw)
    return [
        SpatialRectPrior(
            xmin=float(it["xmin"]),
            ymin=float(it["ymin"]),
            xmax=float(it["xmax"]),
            ymax=float(it["ymax"]),
            boost=float(it.get("boost", 0.15)),
        )
        for it in data
    ]


def main() -> None:
    pa = argparse.ArgumentParser(description="Rosbag + 视觉 JSONL → 语义探索路径")
    pa.add_argument("--survey", required=True, help="survey_zones.json（含 coverage.sites_m）")
    pa.add_argument("--bag", default="", help="ROS1 bag 路径（可选）")
    pa.add_argument("--topic", default="", help="里程计 / 位姿话题，如 /odom")
    pa.add_argument("--detections", default="", help="视觉检测 JSONL（可选，但与 bag 成对使用）")
    pa.add_argument("--out", default=str(_SRC / "exploration" / "data" / "out_semantic_plan.json"))
    pa.add_argument("--cell-size", type=float, default=0.0, help="网格边长（米）；0 表示自动")
    pa.add_argument("--merge-radius", type=float, default=0.45, help="同类物体 DBSCAN 半径（米）")
    pa.add_argument("--random-seed", type=int, default=0)
    pa.add_argument(
        "--priors-json",
        default="",
        help='初始框权重 JSON 数组，如 [{"name":"a","weight":0.8,"applies_to_zone_ids":["A"]}]',
    )
    pa.add_argument(
        "--rect-priors-json",
        default="",
        help='矩形格先验 [{"xmin":0,"ymin":0,"xmax":1,"ymax":1,"boost":0.2}, ...]',
    )
    pa.add_argument("--workspace-margin", type=float, default=-1.0, help=">=0 时用框包络+margin；<0 自动")
    args = pa.parse_args()

    survey_path = Path(args.survey)
    catalog = load_survey_zones(survey_path)
    if args.workspace_margin >= 0:
        ws = workspace_xyxy_from_survey(catalog, margin_m=args.workspace_margin)
    else:
        ws = workspace_xyxy_from_survey(catalog, margin_m=0.6)

    bag = args.bag.strip() or None
    topic = args.topic.strip() or None
    det = args.detections.strip() or None
    if (bag or topic or det) and not (bag and topic and det):
        print("错误：--bag、--topic、--detections 必须三者同时提供或全部省略。", file=sys.stderr)
        sys.exit(2)

    priors = _parse_priors(args.priors_json if args.priors_json else None)
    rect_priors = _parse_rect_priors(args.rect_priors_json if args.rect_priors_json else None)

    result = run_semantic_stack_from_files(
        survey_zones_json=survey_path,
        workspace_xyxy=ws,
        priors=priors,
        bag_path=bag,
        odom_topic=topic,
        detections_jsonl=det,
        spatial_rect_priors=rect_priors or None,
        agg=AggregationParams(),
        path_params=PathPlannerParams(random_seed=args.random_seed),
        cell_size_m=args.cell_size if args.cell_size > 0 else None,
        merge_radius_m=args.merge_radius,
    )

    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "workspace_xyxy": list(ws),
        "survey_input": result.get("survey_input"),
        "cell_size_m": result["cell_size_m"],
        "start_cell_key": list(result["start_cell_key"]),
        "random_seed_used": result["random_seed_used"],
        "zone_weights": result["zone_weights"],
        "per_zone_class_counts": {k: dict(v) for k, v in result["per_zone_class_counts"].items()},
        "rect_weights": {f"{r},{c}": w for (r, c), w in sorted(result["rect_weights"].items())},
        "n_cells": len(result["cells"]),
        "waypoints_xy": [{"x": x, "y": y} for x, y in result["waypoints_xy"]],
        "tour_cell_keys": [list(k) for k in result["tour_cell_keys"]],
    }
    outp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"写入 {outp}")


if __name__ == "__main__":
    main()
