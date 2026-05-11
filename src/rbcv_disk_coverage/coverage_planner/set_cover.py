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
    nnz_tiebreak: bool = True,
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
        nnz_tiebreak: 在以上键仍平局时优先选 CSR 行更短 (**可见格子更少**)
            的候选, 减少「大块低效可见团」在早期被选入的倾向; 不改变近似比阶.
        verbose: 打印进度。

    Returns:
        (selected_indices, n_covered)
    """
    n, m = coverage.shape
    if target_mask is None:
        target_mask = np.ones(m, dtype=bool)
    elif target_mask.dtype != bool:
        target_mask = target_mask.astype(bool)

    tp = tiebreak_priority
    if tp is not None:
        tp = np.asarray(tp, dtype=np.float64).ravel()
        if tp.shape[0] != n:
            raise ValueError(
                "tiebreak_priority length must match number of candidates "
                f"n={n}, got {tp.shape[0]}"
            )

    row_nnz: np.ndarray | None = None
    if nnz_tiebreak:
        ip = coverage.indptr
        row_nnz = (ip[1:] - ip[:-1]).astype(np.int64, copy=False)

    target_total = int(target_mask.sum())
    if target_total == 0:
        return [], 0

    target_required = int(np.ceil(coverage_ratio * target_total))
    covered = np.zeros(m, dtype=bool)
    n_covered_target = 0
    selected: list[int] = []

    indptr = coverage.indptr
    indices = coverage.indices

    def stats_of(i: int) -> tuple[int, int]:
        s, e = indptr[i], indptr[i + 1]
        cols = indices[s:e]
        if cols.size == 0:
            return 0, 0
        in_tar = target_mask[cols]
        cov_here = covered[cols]
        new_cnt = int((in_tar & (~cov_here)).sum())
        ov_cnt = int((in_tar & cov_here).sum())
        return new_cnt, ov_cnt

    def prio_for(i: int) -> float:
        return float(tp[i]) if tp is not None else 0.0

    def nnz_of(i: int) -> int:
        return int(row_nnz[i]) if row_nnz is not None else 0

    def heap_ent(i: int) -> tuple[int, int, float, int, int]:
        nc, ov = stats_of(i)
        ov_eff = ov if overlap_tiebreak else 0
        return (-nc, ov_eff, -prio_for(i), nnz_of(i), i)

    heap: list[tuple[int, int, float, int, int]] = []
    for i in range(n):
        new_cnt, _ov = stats_of(i)
        if new_cnt > 0:
            heapq.heappush(heap, heap_ent(i))

    while heap and n_covered_target < target_required:
        if max_picks is not None and len(selected) >= max_picks:
            break

        stale_ent = heapq.heappop(heap)
        stale_i = int(stale_ent[-1])
        cur_new_ck, _ = stats_of(stale_i)
        if cur_new_ck <= 0:
            continue
        cur_ent = heap_ent(stale_i)
        if cur_ent != stale_ent:
            heapq.heappush(heap, cur_ent)
            continue

        i = stale_i

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
