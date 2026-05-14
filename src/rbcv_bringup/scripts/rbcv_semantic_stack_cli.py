#!/usr/bin/env python3
"""阶段二：转发到 ``run_semantic_stack``。

- 命令行：与 ``exploration/scripts/run_semantic_stack.py`` 相同。
- 经 roslaunch：在节点上设置 ``RBCV_SEMANTIC_FROM_LAUNCH=1`` 及 ``RBCV_*`` 环境变量（见 ``rbcv_stage2_semantic_explore.launch``），由本脚本组装 ``sys.argv``。
- 可选：在输出 JSON 后自动生成并弹出阶段二路径/权重预览图。
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path


def _ensure_src_path() -> Path:
    src = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(src))
    return src


def _render_semantic_preview(plan_json: str, preview_out: str) -> None:
    import matplotlib.pyplot as plt

    payload = json.loads(Path(plan_json).read_text(encoding="utf-8"))
    workspace = payload.get("workspace_xyxy", [0.0, 0.0, 1.0, 1.0])
    x0, y0, x1, y1 = map(float, workspace)

    rect_weights = payload.get("rect_weights", {})
    waypoints = payload.get("waypoints_xy", [])
    zone_weights = payload.get("zone_weights", {})
    cell_size = float(payload.get("cell_size_m", 1.0) or 1.0)

    fig, ax = plt.subplots(figsize=(12, 10))

    if rect_weights:
        vals = [float(v) for v in rect_weights.values()]
        vmin = min(vals)
        vmax = max(vals)
        denom = max(vmax - vmin, 1e-9)
        for key, val in rect_weights.items():
            row_s, col_s = str(key).split(",")
            row = int(row_s)
            col = int(col_s)
            rx = x0 + col * cell_size
            ry = y0 + row * cell_size
            alpha = 0.15 + 0.65 * ((float(val) - vmin) / denom)
            rect = plt.Rectangle(
                (rx, ry),
                cell_size,
                cell_size,
                facecolor="tab:orange",
                edgecolor="none",
                alpha=alpha,
            )
            ax.add_patch(rect)

    if waypoints:
        xs = [float(p["x"]) for p in waypoints]
        ys = [float(p["y"]) for p in waypoints]
        ax.plot(xs, ys, "-o", color="tab:blue", linewidth=2.0, markersize=4, label="语义闭合路径")
        ax.scatter(xs[:1], ys[:1], color="tab:green", s=70, marker="*", label="起点")

    ax.set_title(f"阶段二：语义权重与闭合路径预览（zones={len(zone_weights)}）")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--", alpha=0.25)
    if waypoints:
        ax.legend(loc="best")

    outp = Path(preview_out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(outp, dpi=150)
    print(f"[rbcv_semantic_stack_cli] 已保存阶段二预览图：{outp}")

    if os.environ.get("RBCV_SHOW_PREVIEW", "0").strip().lower() in ("1", "true", "yes", "on"):
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    _ensure_src_path()
    from exploration.scripts.run_semantic_stack import main as stack_main

    preview_out = ""
    show_preview = False

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
        preview_out = os.environ.get("RBCV_PREVIEW_OUT", "").strip()
        show_preview = os.environ.get("RBCV_SHOW_PREVIEW", "0").strip().lower() in (
            "1", "true", "yes", "on"
        )
        sys.argv = argv
    else:
        argv = [sys.argv[0]]
        raw_args = sys.argv[1:]
        i = 0
        while i < len(raw_args):
            a = raw_args[i]
            if a.startswith("__") or ":=" in a:
                i += 1
                continue
            if a == "--preview-out" and i + 1 < len(raw_args):
                preview_out = raw_args[i + 1]
                i += 2
                continue
            if a == "--show-preview":
                show_preview = True
                i += 1
                continue
            argv.append(a)
            i += 1
        sys.argv = argv

    stack_main()

    plan_out = None
    for i, a in enumerate(sys.argv[:-1]):
        if a == "--out":
            plan_out = sys.argv[i + 1]
            break

    if plan_out and (preview_out or show_preview):
        if show_preview:
            os.environ["RBCV_SHOW_PREVIEW"] = "1"
        if not preview_out:
            preview_out = str(Path(plan_out).with_name("rbcv_stage2_preview.png"))
        _render_semantic_preview(plan_out, preview_out)


if __name__ == "__main__":
    main()

