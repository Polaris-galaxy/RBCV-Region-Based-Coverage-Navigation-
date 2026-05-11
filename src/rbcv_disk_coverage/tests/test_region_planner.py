"""region_planner 行为 smoke + tiebreak 测试."""
from __future__ import annotations

import numpy as np

from coverage_planner.map_io import preprocess
from coverage_planner.planner import PlannerConfig
from coverage_planner.region_planner import (
    classify_region,
    config_for_kind,
    plan_partitions,
)


def _square_free(h: int, w: int) -> np.ndarray:
    f = np.ones((h, w), dtype=bool)
    f[0, :] = False
    f[-1, :] = False
    f[:, 0] = False
    f[:, -1] = False
    return f


def test_classify_open_square():
    free = _square_free(40, 40)
    g = preprocess(free)
    s = classify_region(g.free, g.edt, r_px=10.0, region_id=1)
    assert s.kind == "rect_room"
    assert s.rectangularity > 0.85


def test_classify_narrow_corridor():
    """长走廊：高长宽比 + 窄宽度 -> corridor."""
    free = np.zeros((40, 80), dtype=bool)
    free[18:22, 2:78] = True
    g = preprocess(free)
    s = classify_region(g.free, g.edt, r_px=10.0, region_id=2)
    assert s.kind == "corridor"
    assert s.width_ratio < 0.6
    assert s.aspect_ratio >= 3.0


def test_classify_wide_corridor_caught_by_new_threshold():
    """宽 ~1.7r 长走廊：对小 width_ratio 阈值 (2.0) 应判为 corridor。"""
    h, w = 30, 200
    free = np.zeros((h, w), dtype=bool)
    free[7:23, 2:198] = True
    g = preprocess(free)
    s = classify_region(
        g.free, g.edt, r_px=8.0, region_id=10, corridor_width_thresh=2.0,
    )
    assert s.kind == "corridor", (s.kind, s.aspect_ratio, s.width_ratio)
    assert s.aspect_ratio >= 2.5
    assert s.width_ratio < 2.0


def test_plan_partitions_wide_corridor_full_cover():
    """宽 1.5~1.8r 的长走廊：自适应步距应保证全覆盖 (旧固定 √3·r 步距覆盖不到)."""
    h, w = 30, 200
    free = np.zeros((h, w), dtype=bool)
    free[8:22, 2:198] = True
    labels = np.zeros((h, w), dtype=np.int32)
    labels[free] = 1
    res = plan_partitions(
        free_full=free, labels=labels, r_px=8.0,
        base_config=PlannerConfig(
            r=8.0, coverage_ratio=1.0, target_subsample=1,
            random_extra=0, enable_ils=False, verbose=False, seed=0,
        ),
        global_trim=False, parallel_regions=False, verbose=False,
    )
    assert len(res.region_plans) == 1
    rp = res.region_plans[0]
    assert rp.shape.kind == "corridor"
    assert rp.result.coverage_ratio >= 0.999, rp.result.coverage_ratio


def test_classify_narrow_chunk():
    """近似方形但通道窄 -> narrow（不会被误判为 corridor）."""
    free = np.zeros((30, 30), dtype=bool)
    free[12:18, 3:27] = True
    free[3:27, 12:18] = True
    g = preprocess(free)
    s = classify_region(g.free, g.edt, r_px=10.0, region_id=3)
    assert s.kind == "narrow"
    assert s.aspect_ratio < 3.0


def test_config_for_kind_open_disables_medial_reflex():
    base = PlannerConfig(r=10.0, random_extra=500)
    for kk in ("rect_room", "open"):
        cfg = config_for_kind(base, kk)
        assert cfg.use_hex and not cfg.use_medial and not cfg.use_reflex
        assert cfg.random_extra <= 50


def test_plan_partitions_smoke_two_regions():
    h, w = 60, 120
    free = np.zeros((h, w), dtype=bool)
    free[5:55, 5:55] = True
    free[5:55, 65:115] = True
    free[28:32, 55:65] = True
    labels = np.zeros((h, w), dtype=np.int32)
    labels[:, :60] = 1
    labels[:, 60:] = 2

    res = plan_partitions(
        free_full=free,
        labels=labels,
        r_px=8.0,
        base_config=PlannerConfig(
            r=8.0, coverage_ratio=0.9, target_subsample=2,
            random_extra=100, enable_ils=False, verbose=False,
        ),
        verbose=False,
    )
    assert len(res.region_plans) == 2
    assert res.total_circles >= 2
    pts = res.all_points()
    assert pts.shape[0] == res.total_circles


def test_plan_partitions_global_ils_not_worse_than_remove_only():
    """新的 ILS 全局精修至少应不弱于旧版 remove-only (单点删除).

    选择两个紧贴的子矩形分区: 跨区交界附近会有冗余, ILS 能通过 swap_2_for_1 减
    更多. 即使没有可 swap 机会, ILS 也至少能完成 try_remove (不弱于 remove-only).
    """
    h, w = 50, 100
    free = np.zeros((h, w), dtype=bool)
    free[3:47, 3:97] = True
    labels = np.zeros((h, w), dtype=np.int32)
    labels[:, :50] = 1
    labels[:, 50:] = 2

    base = PlannerConfig(
        r=8.0,
        coverage_ratio=1.0,
        target_subsample=1,
        random_extra=80,
        enable_ils=False,
        verbose=False,
        seed=0,
    )
    res_remove = plan_partitions(
        free_full=free, labels=labels, r_px=8.0, base_config=base,
        global_trim=True, global_trim_mode="remove-only",
        trim_max_passes=5,
        verbose=False,
    )
    res_ils = plan_partitions(
        free_full=free, labels=labels, r_px=8.0, base_config=base,
        global_trim=True, global_trim_mode="ils",
        trim_ils_max_iter=8, trim_ils_patience=3, trim_time_limit=20.0,
        verbose=False,
    )
    assert res_ils.total_circles <= res_remove.total_circles, (
        f"ils={res_ils.total_circles} > remove-only={res_remove.total_circles}"
    )
    assert res_ils.trim_stats is not None
    assert res_ils.trim_stats["mode"] == "ils"


def test_plan_coverage_hex_first_full_cover_open_room():
    """开阔房间（远高于 r 的方形）开启 hex_first 后应仍达成 100% 覆盖."""
    from coverage_planner.planner import PlannerConfig, plan_coverage

    free = _square_free(60, 60)
    cfg = PlannerConfig(
        r=8.0, coverage_ratio=1.0, target_subsample=1,
        random_extra=0, enable_ils=False, augment_for_full_cover=True,
        use_hex_first=True, verbose=False, seed=0,
    )
    res = plan_coverage(free, cfg)
    assert res.coverage_ratio >= 0.999, res.coverage_ratio
    assert res.selected_points.shape[0] >= 1


def test_plan_coverage_hex_first_handles_corridor_uncovered():
    """窄走廊场景：hex_first 阶段A几乎没法放 hex，会触发回退到默认流水线."""
    from coverage_planner.planner import PlannerConfig, plan_coverage

    free = np.zeros((40, 80), dtype=bool)
    free[18:22, 2:78] = True
    cfg = PlannerConfig(
        r=8.0, coverage_ratio=1.0, target_subsample=1,
        random_extra=0, enable_ils=False, augment_for_full_cover=True,
        use_hex_first=True, verbose=False, seed=0,
    )
    res = plan_coverage(free, cfg)
    assert res.selected_points.shape[0] >= 1
    assert res.coverage_ratio >= 0.95


def test_plan_partitions_hex_first_smoke_two_regions():
    h, w = 60, 120
    free = np.zeros((h, w), dtype=bool)
    free[5:55, 5:55] = True
    free[5:55, 65:115] = True
    free[28:32, 55:65] = True
    labels = np.zeros((h, w), dtype=np.int32)
    labels[:, :60] = 1
    labels[:, 60:] = 2

    res = plan_partitions(
        free_full=free, labels=labels, r_px=8.0,
        base_config=PlannerConfig(
            r=8.0, coverage_ratio=1.0, target_subsample=1,
            random_extra=80, enable_ils=False,
            use_hex_first=True, verbose=False, seed=0,
        ),
        global_trim=False, parallel_regions=False, verbose=False,
    )
    assert len(res.region_plans) == 2
    assert res.total_circles >= 2


def test_plan_partitions_global_local_preserves_feasible_solution():
    """默认 local 精修不做扰动，应保留可行覆盖并不增加圆数。"""
    h, w = 50, 100
    free = np.zeros((h, w), dtype=bool)
    free[3:47, 3:97] = True
    labels = np.zeros((h, w), dtype=np.int32)
    labels[:, :50] = 1
    labels[:, 50:] = 2

    base = PlannerConfig(
        r=8.0,
        coverage_ratio=1.0,
        target_subsample=1,
        random_extra=80,
        enable_ils=False,
        use_hex_first=True,
        verbose=False,
        seed=0,
    )
    raw = plan_partitions(
        free_full=free,
        labels=labels,
        r_px=8.0,
        base_config=base,
        global_trim=False,
        parallel_regions=False,
        verbose=False,
    )
    local = plan_partitions(
        free_full=free,
        labels=labels,
        r_px=8.0,
        base_config=base,
        global_trim=True,
        global_trim_mode="local",
        parallel_regions=False,
        verbose=False,
    )
    assert local.trim_stats is not None
    assert local.trim_stats["mode"] == "local"
    assert local.total_circles <= raw.total_circles
    assert local.total_circles > 0


def test_plan_partitions_parallel_process_nonempty():
    """多进程后端应能调度各分区并完成规划（冒烟；与串行可比）."""
    h, w = 50, 100
    free = np.zeros((h, w), dtype=bool)
    free[3:47, 3:97] = True
    labels = np.zeros((h, w), dtype=np.int32)
    labels[:, :50] = 1
    labels[:, 50:] = 2

    base = PlannerConfig(
        r=8.0,
        coverage_ratio=1.0,
        target_subsample=2,
        random_extra=80,
        enable_ils=False,
        verbose=False,
        seed=0,
    )
    seq = plan_partitions(
        free_full=free,
        labels=labels,
        r_px=8.0,
        base_config=base,
        global_trim=False,
        parallel_regions=False,
        verbose=False,
    )
    par = plan_partitions(
        free_full=free,
        labels=labels,
        r_px=8.0,
        base_config=base,
        global_trim=False,
        parallel_regions=True,
        parallel_backend="process",
        n_jobs=2,
        verbose=False,
    )
    assert len(par.region_plans) == len(seq.region_plans) == 2
    assert par.raw_total_circles > 0
    assert seq.raw_total_circles > 0
    assert abs(par.raw_total_circles - seq.raw_total_circles) <= 1
