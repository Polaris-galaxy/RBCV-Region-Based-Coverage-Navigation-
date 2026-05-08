"""分区独立覆盖 demo: 每个长方形分区按形状自动选择候选策略.

用法 (默认 r=2.5m, 与项目约定一致)::

    py scripts/demo_regions.py
    py scripts/demo_regions.py ../map_tools/maps/map7.yaml 2.5 \
        ../map_tools/maps/regions_map7.example.json

输出 ``scripts/分区覆盖结果.png`` 与中文控制台明细.
"""
from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAP_TOOLS_MAPS = os.path.abspath(os.path.join(ROOT, "..", "map_tools", "maps"))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from coverage_planner import (  # noqa: E402
    PlannerConfig, plan_partitions,
)
from coverage_planner.map_io import load_ros_map, meters_to_pixels  # noqa: E402
from coverage_planner.region_io import (  # noqa: E402
    load_regions_json,
    rasterize_regions,
)
from coverage_planner.visibility import visible_disk  # noqa: E402

# 默认可调时间上限（秒）。环境变量 RBCV_REGION_ILS_TIME_LIMIT /
# RBCV_TRIM_TIME_LIMIT 显式设置时优先生效；设为 0 表示该维度不限制。
_DEFAULT_REGION_ILS_TIME_LIMIT_S = 180.0
_DEFAULT_GLOBAL_TRIM_TIME_LIMIT_S = 600.0


def _plot(free_full, partition, r_px, save_path, n_rays: int | None = None):
    import matplotlib.pyplot as plt

    h, w = free_full.shape
    fig, ax = plt.subplots(figsize=(12, 12))
    bg = np.where(free_full, 255, 0).astype(np.uint8)
    ax.imshow(bg, cmap="gray", vmin=0, vmax=255,
              origin="upper", interpolation="nearest")

    points = partition.all_points()
    region_ids = partition.all_region_ids()
    cmap = plt.get_cmap("tab10")

    uniq_rids = sorted(set(int(x) for x in region_ids.tolist())) if points.shape[0] else []
    rid_to_color = {rid: cmap(i % 10) for i, rid in enumerate(uniq_rids)}

    rgba = np.zeros((h, w, 4), dtype=np.float32)
    if n_rays is None:
        n_rays = max(64, int(math.ceil(2.0 * math.pi * r_px)))
    alpha = 0.22

    n_pts = points.shape[0]
    if n_pts > 0:
        print(f"[绘图] 按可见性裁剪叠加 {n_pts} 个观测圆…")
    progress_step = max(1, n_pts // 10)
    for k, ((y, x), rid) in enumerate(zip(points, region_ids), start=1):
        rid = int(rid)
        color = rid_to_color[rid]
        mask = visible_disk(free_full, int(y), int(x), r_px, n_rays)
        if not mask.any():
            continue
        prev_a = rgba[mask, 3]
        new_a = prev_a + (1.0 - prev_a) * alpha
        for ch in range(3):
            prev_c = rgba[mask, ch]
            rgba[mask, ch] = (
                prev_c * prev_a + color[ch] * alpha * (1.0 - prev_a)
            ) / np.maximum(new_a, 1e-6)
        rgba[mask, 3] = new_a
        if k % progress_step == 0 or k == n_pts:
            print(f"[绘图] 进度 {k}/{n_pts}")

    ax.imshow(rgba, origin="upper", interpolation="nearest", zorder=1)

    legend_seen: set[int] = set()
    for (y, x), rid in zip(points, region_ids):
        rid = int(rid)
        color = rid_to_color[rid]
        if rid not in legend_seen:
            ax.scatter(
                [x], [y], s=26, c=[color], marker="x",
                linewidths=1.4, label=f"Region {rid}", zorder=3,
            )
            legend_seen.add(rid)
        else:
            ax.scatter(
                [x], [y], s=26, c=[color], marker="x",
                linewidths=1.4, zorder=3,
            )

    raw_n = partition.raw_total_circles
    final_n = partition.total_circles
    if partition.trim_stats is not None and raw_n != final_n:
        title = (
            f"Per-region coverage (visibility clipped) | "
            f"global trim: {raw_n} -> {final_n}"
        )
    else:
        title = (
            f"Per-region coverage (visibility clipped) | "
            f"total disks = {final_n}"
        )
    ax.set_title(title)
    ax.set_xlabel("x (px)")
    ax.set_ylabel("y (px)")

    if legend_seen:
        ax.legend(loc="upper right", fontsize=8)
    ax.set_aspect("equal")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)


def main():
    yaml_path = os.path.join(MAP_TOOLS_MAPS, "map7.yaml")
    r_meters = 2.5
    regions_path = os.path.join(MAP_TOOLS_MAPS, "regions_map7.example.json")

    if len(sys.argv) >= 2:
        yaml_path = sys.argv[1]
    if len(sys.argv) >= 3:
        r_meters = float(sys.argv[2])
    if len(sys.argv) >= 4:
        regions_path = sys.argv[3]

    print(f"[演示] 地图 YAML        = {yaml_path}")
    print(f"[演示] 覆盖半径 r       = {r_meters} 米")
    print(f"[演示] 分区 JSON        = {regions_path}")

    free, info = load_ros_map(
        yaml_path,
        keep_largest_component=False,
        clean_map_verbose=True,
    )
    spec = load_regions_json(regions_path)
    labels = rasterize_regions(spec, info, free.shape)
    in_region = int(((labels >= 1) & free).sum())
    if in_region < 100:
        print(
            "[demo] ERROR: regions JSON does not overlap free space.\n"
            f"        overlap_pixels = {in_region}\n"
            "        Fix your rectangles (draw_regions.py) or provide a correct "
            "regions JSON.\n"
            "        (We no longer fallback to whole-map mode, to avoid misleading "
            "results.)"
        )
        raise SystemExit(2)
    r_px = meters_to_pixels(r_meters, info.resolution)
    print(
        f"[演示] 地图尺寸={free.shape}，r（像素）={r_px:.2f}，"
        f"有效分区像素={int(((labels >= 1) & free).sum())}"
    )

    fast = os.environ.get("RBCV_FAST", "0").strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    geo_n_rays = max(64, int(math.ceil(2.0 * math.pi * r_px)))
    nr_env = os.environ.get("RBCV_N_RAYS", "").strip()
    if nr_env:
        n_rays_plan = max(16, int(nr_env))
    elif fast:
        n_rays_plan = min(128, geo_n_rays)
    else:
        n_rays_plan = None

    hex_first_env = os.environ.get("RBCV_HEX_FIRST", "1").strip().lower()
    use_hex_first = hex_first_env in ("1", "true", "yes", "on")
    hex_full_ils = os.environ.get("RBCV_HEX_FIRST_FULL_ILS", "0").strip().lower() in (
        "1", "true", "yes", "on",
    )
    # hex_first 下少撒随机点，避免 ILS 修复阶段引入「靠边一团」的冗余圆。
    random_extra_eff = 120 if use_hex_first else 200

    base = PlannerConfig(
        r=r_px,
        coverage_ratio=1.0,
        # 必须为 1：步长>1 时规划器只保证稀疏采样格被覆盖，
        # 与「可见圆盘」整张叠图对照会出现大片未上色白区（误以为未覆盖）。
        target_subsample=1,
        min_corridor=2.0,
        augment_for_full_cover=True,
        random_extra=random_extra_eff,
        n_rays=n_rays_plan,
        enable_ils=True,
        ils_max_iter=40,
        ils_patience=10,
        overlap_tiebreak=True,
        use_hex_first=use_hex_first,
        hex_first_use_full_ils=hex_full_ils,
        verbose=False,
        seed=0,
    )
    print(
        f"[演示] 候选流水线 = "
        f"{'hex_first（六边形铺底+残余补圆）' if use_hex_first else '默认（按形状预分类）'}"
        f"（RBCV_HEX_FIRST，默认 1）；"
        f"区内精修 = {'完整 ILS（扰动）' if hex_full_ils else '仅局部 search（默认，保布局）'}"
        f"（RBCV_HEX_FIRST_FULL_ILS）"
    )

    best = os.environ.get("RBCV_BEST", "0").strip().lower() in ("1", "true", "yes", "on")
    trim_mode = os.environ.get("RBCV_TRIM_MODE", "local").strip().lower()
    soft_factor = float(os.environ.get("RBCV_SOFT_FACTOR", "0.10"))
    soft_iso = int(os.environ.get("RBCV_SOFT_ISO_PX", "64"))
    ils_iter = int(os.environ.get("RBCV_TRIM_ILS_ITER", "30"))
    ils_pat = int(os.environ.get("RBCV_TRIM_ILS_PATIENCE", "8"))
    swap3_attempts = int(os.environ.get("RBCV_TRIM_SWAP3_ATTEMPTS", "120000"))

    trim_tl_env = os.environ.get("RBCV_TRIM_TIME_LIMIT", "").strip()
    trim_tl_explicit = trim_tl_env != ""
    time_limit = float(trim_tl_env) if trim_tl_explicit else 0.0

    # 各分区内 plan_coverage 的 ILS 墙钟上限；未设置环境变量时用脚本默认（见文件头常量）。
    reg_ils_env = os.environ.get("RBCV_REGION_ILS_TIME_LIMIT", "").strip()
    reg_ils_explicit = reg_ils_env != ""
    if reg_ils_explicit:
        _rv = float(reg_ils_env)
        region_ils_time_limit: float | None = None if _rv <= 0 else _rv
    else:
        region_ils_time_limit = None

    trim_enable_swap3 = True
    if fast and not os.environ.get("RBCV_TRIM_SWAP3_ATTEMPTS"):
        trim_enable_swap3 = False

    if fast:
        base = replace(
            base,
            enable_ils=False,
            ils_max_iter=0,
            ils_patience=0,
            random_extra=80,
        )
        ils_iter = min(ils_iter, 12)
        ils_pat = min(ils_pat, 4)
        swap3_attempts = min(swap3_attempts, 20000)
        print(
            "[演示] RBCV_FAST=1：降低射线数、关闭分区 ILS、缩短全局 trim；"
            "质量可能下降。依赖已含 numba（requirements.txt），可加速可见性。"
        )

    if not trim_tl_explicit:
        if fast:
            time_limit = 180.0
        elif best:
            time_limit = 600.0
        else:
            time_limit = _DEFAULT_GLOBAL_TRIM_TIME_LIMIT_S

    if not reg_ils_explicit:
        region_ils_time_limit = None if fast else _DEFAULT_REGION_ILS_TIME_LIMIT_S

    if not fast and region_ils_time_limit is not None:
        base = replace(base, ils_time_limit=region_ils_time_limit)

    if best:
        trim_mode = "regreedy"
        soft_factor = max(soft_factor, 0.12)
        soft_iso = max(soft_iso, 64)
        ils_iter = max(ils_iter, 60)
        ils_pat = max(ils_pat, 12)
        swap3_attempts = max(swap3_attempts, 300000)
        if time_limit <= 0:
            time_limit = 600.0

    print(
        f"[演示] 全局精修模式 = {trim_mode}（RBCV_TRIM_MODE；best={best}），"
        f"软化={soft_factor:.2f}*r（RBCV_SOFT_FACTOR），"
        f"孤立阈值={soft_iso}px（RBCV_SOFT_ISO_PX），"
        f"ILS={ils_iter}/{ils_pat}（RBCV_TRIM_ILS_ITER/PATIENCE），"
        f"swap3_max={swap3_attempts}（RBCV_TRIM_SWAP3_ATTEMPTS），"
        f"time_limit={'不限制' if time_limit <= 0 else f'{time_limit}s'}"
        f"（RBCV_TRIM_TIME_LIMIT）"
    )
    if fast:
        print("[演示] 分区内 ILS = 已关闭（RBCV_FAST）")
    else:
        _rils = (
            "不限制"
            if region_ils_time_limit is None
            else f"{region_ils_time_limit}s"
        )
        print(
            f"[演示] 分区内 ILS 时间上限 = {_rils}（RBCV_REGION_ILS_TIME_LIMIT；"
            f"默认 {_DEFAULT_REGION_ILS_TIME_LIMIT_S:.0f}s；0=不限制）"
        )
        if time_limit <= 0:
            print(
                "[演示] 提示：全局精修为不限制（RBCV_TRIM_TIME_LIMIT=0）；"
                "百万级格可能极慢。"
            )

    par_backend = os.environ.get("RBCV_PAR_BACKEND", "process").strip().lower()
    print(
        "[演示] 分区并行后端 = "
        f"{par_backend}（RBCV_PAR_BACKEND；process=多进程吃满多核，thread=通常看不出多核）"
    )

    t0 = time.time()
    partition = plan_partitions(
        free_full=free,
        labels=labels,
        r_px=r_px,
        base_config=base,
        global_trim=True,
        global_trim_mode=trim_mode,
        trim_ils_max_iter=ils_iter,
        trim_ils_patience=ils_pat,
        trim_enable_swap3=trim_enable_swap3,
        trim_swap3_max_attempts=swap3_attempts,
        trim_time_limit=(None if time_limit <= 0 else time_limit),
        trim_soft_target_factor=soft_factor,
        trim_soft_isolated_max_area_px=soft_iso,
        parallel_regions=True,
        parallel_backend=par_backend,
        parallel_log=True,
        verbose=True,
    )
    elapsed = time.time() - t0

    print()
    print("=" * 72)
    if partition.trim_stats is not None:
        print(
            f"[演示] 汇总：圆盘数 {partition.raw_total_circles} → "
            f"{partition.total_circles}（全局精修共减 "
            f"{partition.trim_stats['removed']} 个），用时 {elapsed:.1f} 秒"
        )
    else:
        print(
            f"[演示] 汇总：圆盘总数 = {partition.total_circles}  "
            f"（用时 {elapsed:.1f} 秒）"
        )
    print("=" * 72)
    for rp in partition.region_plans:
        s = rp.shape
        n = rp.result.selected_points.shape[0]
        print(
            f"  分区{rp.region_id:>2} 类型={s.kind:<6} "
            f"可走像素={s.free_px:>7d} 长方形程度={s.rectangularity:.2f} "
            f"宽度比={s.width_ratio:.2f} → 圆盘数={n} "
            f"覆盖率={100*rp.result.coverage_ratio:.2f}%"
        )

    save_path = os.path.join(HERE, "分区覆盖结果.png")
    try:
        _plot(
            free,
            partition,
            r_px,
            save_path,
            n_rays=n_rays_plan if n_rays_plan is not None else geo_n_rays,
        )
        print(f"[演示] 已保存结果图：{save_path}")
    except Exception as e:
        print(f"[演示] 绘图失败：{e}")


if __name__ == "__main__":
    main()
