"""贪心 Set Cover (lazy max-heap) + 可达性检查。

经典 Johnson 1974 贪心: 每轮挑覆盖"未覆盖元素"最多的子集。
近似比 ln|U| + 1。Lazy 实现: 堆中存陈旧增益, 弹出时再校验,
若已过期则按当前真实值压回, 以避免每选一次都做 O(|C|) 全量更新。

输入:
    coverage: csr_matrix[|C|, |U|], bool, ``True`` = 候选 i 覆盖元素 j。
    target_mask: ``np.ndarray[|U|], bool``, 必须被覆盖的元素子集; 默认全 True。

输出:
    selected: ``list[int]``, 选中候选索引的顺序。
    covered_count: 总共覆盖到的目标元素数。
"""
from __future__ import annotations

import heapq

import numpy as np
from scipy.sparse import csr_matrix


def find_unreachable_targets(
    coverage: csr_matrix, target_mask: np.ndarray | None = None
) -> np.ndarray:
    """返回 ``target_mask`` 中没有任何候选可覆盖的目标索引。

    用于"全覆盖"模式前的可行性诊断: 这些目标不可能被任何候选盖到,
    若不为空则需补全候选, 否则 ``coverage_ratio=1.0`` 必然失败。
    """
    n, m = coverage.shape
    if target_mask is None:
        target_mask = np.ones(m, dtype=bool)
    col_count = np.asarray((coverage > 0).sum(axis=0)).ravel()
    unreachable = target_mask & (col_count == 0)
    return np.where(unreachable)[0]


def greedy_set_cover(
    coverage: csr_matrix,
    target_mask: np.ndarray | None = None,
    coverage_ratio: float = 1.0,
    max_picks: int | None = None,
    overlap_tiebreak: bool = True,
    tiebreak_priority: np.ndarray | None = None,
    verbose: bool = False,
) -> tuple[list[int], int]:
    """贪心选取候选, 直到 (覆盖率达标 / 候选耗尽 / 达到 max_picks)。

    Args:
        coverage: 覆盖关系稀疏矩阵 ``[N, M]``。
        target_mask: 长度 M 的 bool, ``True`` 表示需要覆盖。
            ``None`` 视为全部需覆盖。
        coverage_ratio: 目标覆盖比例 (0~1]. 1.0 即必须覆盖全部目标。
        max_picks: 最多挑几个 (None=无上限)。
        overlap_tiebreak: 当多个候选新增覆盖相同时, 优先选与已覆盖区域
            重叠最少的 (减少边缘 / 交叉口处的密集叠加)。不影响最终覆
            盖率, 仅作 tie-breaker.
        tiebreak_priority: 长度 N 的非负数组, **数值越大越优先** (例如圆心处
            欧氏距离变换 EDT)。在 ``new_cnt`` 与 (若启用) ``overlap`` 均
            相同时作为第三级键; ``None`` 表示不使用.
        verbose: 打印进度。

    Returns:
        (selected_indices, n_covered)
    """
    n, m = coverage.shape
    if target_mask is None:
        target_mask = np.ones(m, dtype=bool)
    elif target_mask.dtype != bool:
        target_mask = target_mask.astype(bool)

    if tiebreak_priority is not None:
        tp = np.asarray(tiebreak_priority, dtype=np.float64).ravel()
        if tp.shape[0] != n:
            raise ValueError(
                "tiebreak_priority length must match number of candidates "
                f"n={n}, got {tp.shape[0]}"
            )
    else:
        tp = None

    target_total = int(target_mask.sum())
    if target_total == 0:
        return [], 0

    target_required = int(np.ceil(coverage_ratio * target_total))
    covered = np.zeros(m, dtype=bool)
    n_covered_target = 0
    selected: list[int] = []

    indptr = coverage.indptr
    indices = coverage.indices

    def prio_for(i: int) -> float:
        return float(tp[i]) if tp is not None else 0.0

    def stats_of(i: int) -> tuple[int, int]:
        """returns (new_covered, overlap_with_already_covered)."""
        s, e = indptr[i], indptr[i + 1]
        cols = indices[s:e]
        if cols.size == 0:
            return 0, 0
        in_target = target_mask[cols]
        cov_here = covered[cols]
        new_cnt = int((in_target & (~cov_here)).sum())
        ov_cnt = int((in_target & cov_here).sum())
        return new_cnt, ov_cnt

    heap: list[tuple[int, int, float, int]] = []
    for i in range(n):
        new_cnt, ov_cnt = stats_of(i)
        if new_cnt > 0:
            pri = prio_for(i)
            heapq.heappush(
                heap,
                (-new_cnt, ov_cnt if overlap_tiebreak else 0, -pri, i),
            )

    while heap and n_covered_target < target_required:
        if max_picks is not None and len(selected) >= max_picks:
            break

        neg_g, stale_ov, stale_neg_pri, i = heapq.heappop(heap)
        if -neg_g <= 0:
            break
        cur_new, cur_ov = stats_of(i)
        if cur_new == 0:
            continue
        cur_pri = prio_for(i)
        cur_neg_pri = -cur_pri
        if cur_new < -neg_g or (
            overlap_tiebreak and cur_new == -neg_g and cur_ov > stale_ov
        ) or (
            tp is not None
            and cur_new == -neg_g
            and (not overlap_tiebreak or cur_ov == stale_ov)
            and cur_neg_pri > stale_neg_pri
        ):
            heapq.heappush(
                heap,
                (-cur_new, cur_ov if overlap_tiebreak else 0, -cur_pri, i),
            )
            continue

        s, e = indptr[i], indptr[i + 1]
        cols = indices[s:e]
        new_mask = target_mask[cols] & (~covered[cols])
        newly_covered = cols[new_mask]
        covered[newly_covered] = True
        n_covered_target += int(newly_covered.size)
        selected.append(int(i))

        if verbose and len(selected) % 20 == 0:
            pct = 100.0 * n_covered_target / target_total
            print(
                f"[贪心覆盖] 进度：已选圆盘 {len(selected)} 个，"
                f"已覆盖目标 {n_covered_target}/{target_total}（{pct:.1f}%）"
            )

    return selected, n_covered_target
