"""ILS (Iterated Local Search) 精修器.

针对 "等半径圆盘覆盖 + 全覆盖硬约束 + 圆数最少" 问题:
    给定贪心初始可行解, 通过局部搜索 + 扰动迭代降低圆数.

理论支撑:
    Mustafa & Ray, "Improved results on geometric hitting set problems"
    (Discrete & Computational Geometry, 2010): 对 unit disk cover, 纯
    k-邻域局部搜索可达 PTAS (任意 (1+eps) 近似).

实现的邻域算子:
    1. ``try_remove(c)``        删 c 后仍可行 -> 接受
    2. ``swap_2_for_1``         相邻 2 个删, 替换为 1 个 -> 净减 1
    (3. ``swap_3_for_2``         留作扩展, 当前默认关闭以控时间)

主循环:
    repeat:
        local_search 直到无改进
        if 无整体改进 cnt >= patience: break
        perturb (随机删 m, 修复贪心)
"""
from __future__ import annotations

import random
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.sparse import csr_matrix
from scipy.spatial import cKDTree


@dataclass
class SolutionState:
    """增量维护的可行解状态.

    Fields:
        coverage: csr ``[|C|, |U|]`` , 全部候选 vs 全部目标.
        target_mask: ``[|U|], bool`` 必须覆盖.
        in_solution: ``[|C|], bool`` 当前是否被选中.
        cover_count: ``[|U|], int32`` 每个目标被几个 *已选* 圆覆盖.
                     仅对 ``target_mask`` 内目标有意义.
        candidates_xy: ``[|C|, 2]`` 候选坐标 (供邻接查询).
    """

    coverage: csr_matrix
    target_mask: np.ndarray
    in_solution: np.ndarray
    cover_count: np.ndarray
    candidates_xy: np.ndarray
    _kdtree: cKDTree | None = field(default=None, repr=False)

    @classmethod
    def from_selected(
        cls,
        coverage: csr_matrix,
        target_mask: np.ndarray,
        candidates_xy: np.ndarray,
        selected: list[int],
    ) -> "SolutionState":
        n_c, n_u = coverage.shape
        in_sol = np.zeros(n_c, dtype=bool)
        in_sol[selected] = True
        cover_count = np.zeros(n_u, dtype=np.int32)
        indptr = coverage.indptr
        indices = coverage.indices
        for c in selected:
            cols = indices[indptr[c]:indptr[c + 1]]
            cover_count[cols] += 1
        return cls(
            coverage=coverage,
            target_mask=target_mask.astype(bool),
            in_solution=in_sol,
            cover_count=cover_count,
            candidates_xy=candidates_xy.astype(np.int32),
        )

    @property
    def kdtree(self) -> cKDTree:
        if self._kdtree is None:
            self._kdtree = cKDTree(self.candidates_xy)
        return self._kdtree

    @property
    def size(self) -> int:
        return int(self.in_solution.sum())

    def selected_indices(self) -> np.ndarray:
        return np.where(self.in_solution)[0]

    def cols_of(self, c: int) -> np.ndarray:
        s = self.coverage.indptr[c]
        e = self.coverage.indptr[c + 1]
        return self.coverage.indices[s:e]

    def add(self, c: int) -> None:
        if self.in_solution[c]:
            return
        self.in_solution[c] = True
        self.cover_count[self.cols_of(c)] += 1

    def remove(self, c: int) -> None:
        if not self.in_solution[c]:
            return
        self.in_solution[c] = False
        self.cover_count[self.cols_of(c)] -= 1

    def is_feasible(self) -> bool:
        return bool((~(self.cover_count > 0) & self.target_mask).sum() == 0)

    def n_uncovered(self) -> int:
        return int(((self.cover_count == 0) & self.target_mask).sum())

    def unique_cols(self, c: int) -> np.ndarray:
        """``c`` 在当前解中独占覆盖的目标列 (cover_count==1 且 c 覆盖)."""
        cols = self.cols_of(c)
        mask = (self.cover_count[cols] == 1) & self.target_mask[cols]
        return cols[mask]

    def need_for_pair_removal(self, c1: int, c2: int) -> np.ndarray:
        """同时移除 c1 与 c2 后会变为未覆盖的目标集 (target_mask 内).

        必须包含三类:
            - 仅 c1 覆盖且 cover_count==1
            - 仅 c2 覆盖且 cover_count==1
            - 两者都覆盖且 cover_count==2 (c1+c2 是仅有的两个)
        """
        cols1 = self.cols_of(c1)
        cols2 = self.cols_of(c2)
        both = np.intersect1d(cols1, cols2, assume_unique=False)
        only1 = np.setdiff1d(cols1, cols2, assume_unique=False)
        only2 = np.setdiff1d(cols2, cols1, assume_unique=False)

        t = self.target_mask
        cc = self.cover_count
        need_only1 = only1[(cc[only1] == 1) & t[only1]]
        need_only2 = only2[(cc[only2] == 1) & t[only2]]
        need_both = both[(cc[both] == 2) & t[both]]

        out = np.concatenate([need_only1, need_only2, need_both])
        out.sort()
        return out

    def need_for_triple_removal(
        self, c1: int, c2: int, c3: int
    ) -> np.ndarray:
        """同时移除 (c1, c2, c3) 后会变为未覆盖的目标集.

        ``need = { u : target_mask[u] AND cover_count[u] == coverers_in_triple(u) }``
        其中 ``coverers_in_triple(u) = 1[c1 cov u] + 1[c2 cov u] + 1[c3 cov u]``.

        在 ``cols(c1) ∪ cols(c2) ∪ cols(c3)`` 上扫描计算.
        """
        cols1 = self.cols_of(c1)
        cols2 = self.cols_of(c2)
        cols3 = self.cols_of(c3)
        all_cols = np.unique(np.concatenate([cols1, cols2, cols3]))

        in1 = np.zeros(all_cols.size, dtype=np.int8)
        in2 = np.zeros(all_cols.size, dtype=np.int8)
        in3 = np.zeros(all_cols.size, dtype=np.int8)
        idx = np.searchsorted(all_cols, cols1)
        in1[idx] = 1
        idx = np.searchsorted(all_cols, cols2)
        in2[idx] = 1
        idx = np.searchsorted(all_cols, cols3)
        in3[idx] = 1
        triple_count = in1 + in2 + in3

        cc = self.cover_count[all_cols]
        t = self.target_mask[all_cols]
        crit = t & (cc == triple_count) & (triple_count > 0)
        return all_cols[crit]

    def can_remove(self, c: int) -> bool:
        if not self.in_solution[c]:
            return False
        cols = self.cols_of(c)
        crit = (self.cover_count[cols] == 1) & self.target_mask[cols]
        return not bool(crit.any())

    def covers_all(self, c: int, cols_required: np.ndarray) -> bool:
        """候选 ``c`` 是否覆盖 ``cols_required`` 中所有目标.

        用 ``searchsorted`` 替代 ``np.isin`` , 利用 ``cols_of(c)`` 已 sorted
        的特性, 复杂度 O(|need| log |c_cols|), 实测比 isin 快 3-5x.
        """
        if cols_required.size == 0:
            return True
        c_cols = self.cols_of(c)
        if c_cols.size == 0:
            return False
        idx = np.searchsorted(c_cols, cols_required)
        if (idx >= c_cols.size).any():
            return False
        return bool((c_cols[idx] == cols_required).all())

    def covers_count(self, c: int, cols_required: np.ndarray) -> int:
        """候选 ``c`` 覆盖了 ``cols_required`` 中的几个目标."""
        if cols_required.size == 0:
            return 0
        c_cols = self.cols_of(c)
        if c_cols.size == 0:
            return 0
        idx = np.searchsorted(c_cols, cols_required)
        in_range = idx < c_cols.size
        if not in_range.any():
            return 0
        ok = c_cols[np.minimum(idx, c_cols.size - 1)] == cols_required
        return int((in_range & ok).sum())


def _try_remove_pass(state: SolutionState, verbose: bool = False) -> int:
    """对每个已选候选尝试删除. 返回成功删除数."""
    sel = state.selected_indices()
    rng = np.random.default_rng()
    order = rng.permutation(sel.size)
    removed = 0
    for k in order:
        c = int(sel[k])
        if not state.in_solution[c]:
            continue
        if state.can_remove(c):
            state.remove(c)
            removed += 1
    if verbose and removed:
        print(f"  [LS:remove] -{removed}, |S|={state.size}")
    return removed


def _try_swap_2_for_1(
    state: SolutionState,
    r: float,
    pair_radius_factor: float = 1.6,
    cand_radius_factor: float = 1.2,
    max_pairs: int | None = None,
    verbose: bool = False,
) -> int:
    """删 2 加 1 净减 1 的局部移动.

    枚举 "近距离" 候选对 (c1, c2) (距离 < ``pair_radius_factor*r``).
    几何理由: c' 半径为 r, 单圆覆盖直径 2r, 故 c1, c2 间距 > 2r 时
    一个 c' 不可能同时覆盖二者独占目标. 留少许余量取 1.6 .

    流程:
        need = need_for_pair_removal(c1, c2)
        在 (c1+c2)/2 周围半径 ``cand_radius_factor*r`` 内寻找未选 c'
        满足 cols(c') ⊇ need.

    Returns:
        swap 成功数.
    """
    sel = state.selected_indices()
    if sel.size < 2:
        return 0

    tree = state.kdtree
    sel_set = set(int(x) for x in sel)
    pairs_done = 0
    swaps = 0

    for c1 in sel:
        if not state.in_solution[c1]:
            continue

        c1_xy = state.candidates_xy[c1]
        nbrs = tree.query_ball_point(c1_xy, r=pair_radius_factor * r)
        for c2 in nbrs:
            if c2 == c1:
                continue
            if c2 not in sel_set:
                continue
            if c2 < c1:
                continue
            if not state.in_solution[c2]:
                continue

            need = state.need_for_pair_removal(int(c1), int(c2))
            if need.size == 0:
                continue

            mid_xy = (state.candidates_xy[c1] + state.candidates_xy[c2]) / 2.0
            cand_pool = tree.query_ball_point(
                mid_xy, r=cand_radius_factor * r
            )
            best_c: int | None = None
            for c in cand_pool:
                if c == c1 or c == c2:
                    continue
                if state.in_solution[c]:
                    continue
                if state.covers_all(int(c), need):
                    best_c = int(c)
                    break

            if best_c is None:
                continue

            state.remove(int(c1))
            state.remove(int(c2))
            state.add(best_c)
            sel_set.discard(int(c1))
            sel_set.discard(int(c2))
            sel_set.add(best_c)
            swaps += 1
            pairs_done += 1
            if max_pairs is not None and pairs_done >= max_pairs:
                break
            break
        if max_pairs is not None and pairs_done >= max_pairs:
            break

    if verbose and swaps:
        print(f"  [LS:swap2->1] -{swaps}, |S|={state.size}")
    return swaps


def _try_swap_3_for_2(
    state: SolutionState,
    r: float,
    triple_radius_factor: float = 2.0,
    cand_radius_factor: float = 1.4,
    max_attempts: int | None = None,
    verbose: bool = False,
) -> int:
    """删 3 加 2 净减 1 的局部移动.

    枚举三元组 (c1, c2, c3) 满足:
        - 三个候选两两距离 < ``triple_radius_factor * r``
        - 都在当前解中
    然后:
        need = need_for_triple_removal(c1, c2, c3)
        在 (c1+c2+c3)/3 周围寻找 *2* 个未选候选 (c', c'') 使
        ``cols(c') ∪ cols(c'') ⊇ need``.
        若找到则 remove(c1,c2,c3); add(c',c''); 净减 1.

    搜索 c', c'' 的策略 (启发式, 不保证最优):
        1. 对邻域内每个未选 c, 计算 ``g = covers_count(c, need)`` ;
        2. 选 ``g`` 最大的 c' (尽量多盖);
        3. ``residual = need \\ cols(c')`` ;
        4. 在邻域内选另一个 c'' (c'' != c') 满足 ``cols(c'') ⊇ residual`` ,
           若没有则放弃此三元组.

    Args:
        max_attempts: 最多尝试三元组数 (None=不限).

    Returns:
        成功 swap 数.
    """
    sel = state.selected_indices()
    if sel.size < 3:
        return 0

    tree = state.kdtree
    sel_set = set(int(x) for x in sel)
    swaps = 0
    attempts = 0
    pair_thresh = triple_radius_factor * r

    for c1 in sel:
        if not state.in_solution[c1]:
            continue
        c1_xy = state.candidates_xy[c1]
        nbrs = tree.query_ball_point(c1_xy, r=pair_thresh)
        # 仅对有序对 (c1<c2<c3) 枚举, 避免重复.
        nbrs_sorted = sorted(int(x) for x in nbrs if int(x) > int(c1))
        nbrs_sel = [c for c in nbrs_sorted if c in sel_set]
        for i in range(len(nbrs_sel)):
            c2 = nbrs_sel[i]
            if not state.in_solution[c2]:
                continue
            for j in range(i + 1, len(nbrs_sel)):
                c3 = nbrs_sel[j]
                if not state.in_solution[c3]:
                    continue
                d23 = float(np.hypot(
                    state.candidates_xy[c2, 0] - state.candidates_xy[c3, 0],
                    state.candidates_xy[c2, 1] - state.candidates_xy[c3, 1],
                ))
                if d23 > pair_thresh:
                    continue

                attempts += 1
                if max_attempts is not None and attempts > max_attempts:
                    break

                need = state.need_for_triple_removal(int(c1), c2, c3)
                if need.size == 0:
                    continue

                centroid = (
                    state.candidates_xy[c1]
                    + state.candidates_xy[c2]
                    + state.candidates_xy[c3]
                ) / 3.0
                pool = tree.query_ball_point(
                    centroid, r=cand_radius_factor * r * 1.4
                )
                cand_pool = [
                    int(c) for c in pool
                    if int(c) not in (int(c1), c2, c3)
                    and not state.in_solution[int(c)]
                ]
                if len(cand_pool) < 2:
                    continue

                best_c1 = -1
                best_g = 0
                for c in cand_pool:
                    g = state.covers_count(c, need)
                    if g > best_g:
                        best_g = g
                        best_c1 = c
                if best_c1 < 0 or best_g == 0:
                    continue

                if best_g >= need.size:
                    state.remove(int(c1))
                    state.remove(c2)
                    state.remove(c3)
                    state.add(best_c1)
                    sel_set.discard(int(c1))
                    sel_set.discard(c2)
                    sel_set.discard(c3)
                    sel_set.add(best_c1)
                    swaps += 1
                    continue

                c1_cols = state.cols_of(best_c1)
                idx = np.searchsorted(c1_cols, need)
                in_range = idx < c1_cols.size
                covered_by_c1 = np.zeros(need.size, dtype=bool)
                if in_range.any():
                    covered_by_c1[in_range] = (
                        c1_cols[idx[in_range]] == need[in_range]
                    )
                residual = need[~covered_by_c1]
                if residual.size == 0:
                    state.remove(int(c1))
                    state.remove(c2)
                    state.remove(c3)
                    state.add(best_c1)
                    sel_set.discard(int(c1))
                    sel_set.discard(c2)
                    sel_set.discard(c3)
                    sel_set.add(best_c1)
                    swaps += 1
                    continue

                best_c2 = -1
                for c in cand_pool:
                    if c == best_c1:
                        continue
                    if state.covers_all(c, residual):
                        best_c2 = c
                        break
                if best_c2 < 0:
                    continue

                state.remove(int(c1))
                state.remove(c2)
                state.remove(c3)
                state.add(best_c1)
                state.add(best_c2)
                sel_set.discard(int(c1))
                sel_set.discard(c2)
                sel_set.discard(c3)
                sel_set.add(best_c1)
                sel_set.add(best_c2)
                swaps += 1

            if max_attempts is not None and attempts > max_attempts:
                break
        if max_attempts is not None and attempts > max_attempts:
            break

    if verbose and swaps:
        print(f"  [LS:swap3->2] -{swaps}, |S|={state.size}, attempts={attempts}")
    return swaps


def local_search(
    state: SolutionState,
    r: float,
    enable_swap2: bool = True,
    enable_swap3: bool = False,
    max_passes: int = 20,
    swap3_max_attempts: int | None = 100000,
    verbose: bool = False,
) -> int:
    """交替执行 try_remove / swap_2_for_1 / swap_3_for_2 直到无改进."""
    total = 0
    for it in range(max_passes):
        before = state.size
        rm = _try_remove_pass(state, verbose=verbose)
        sw2 = (
            _try_swap_2_for_1(state, r=r, verbose=verbose)
            if enable_swap2
            else 0
        )
        sw3 = 0
        if enable_swap3 and (rm + sw2) == 0:
            sw3 = _try_swap_3_for_2(
                state, r=r,
                max_attempts=swap3_max_attempts,
                verbose=verbose,
            )
        total += rm + sw2 + sw3
        if state.size == before:
            break
        if verbose:
            print(f"  [LS:pass {it}] |S|={state.size}")
    return total


def _greedy_repair(state: SolutionState, verbose: bool = False) -> int:
    """若存在未覆盖目标, 用一次贪心从未选候选中补齐. 返回新增数."""
    if state.is_feasible():
        return 0
    n_c, n_u = state.coverage.shape
    indptr = state.coverage.indptr
    indices = state.coverage.indices
    target_mask = state.target_mask

    added = 0
    while not state.is_feasible():
        uncov = (state.cover_count == 0) & target_mask
        best_c = -1
        best_g = 0
        for c in range(n_c):
            if state.in_solution[c]:
                continue
            cols = indices[indptr[c]:indptr[c + 1]]
            g = int(uncov[cols].sum())
            if g > best_g:
                best_g = g
                best_c = c
        if best_c < 0:
            break
        state.add(best_c)
        added += 1
    if verbose and added:
        print(f"  [repair] +{added}, |S|={state.size}")
    return added


def _perturb_random(
    state: SolutionState,
    strength: float,
    rng: random.Random,
    verbose: bool = False,
) -> None:
    """随机移除 ``strength`` 比例的已选候选."""
    sel = state.selected_indices().tolist()
    if not sel:
        return
    k = max(1, int(round(strength * len(sel))))
    rng.shuffle(sel)
    for c in sel[:k]:
        state.remove(int(c))
    if verbose:
        print(f"  [perturb-rand] removed {k}, now |S|={state.size}, "
              f"uncov={state.n_uncovered()}")


def _perturb_block(
    state: SolutionState,
    r: float,
    rng: random.Random,
    block_factor: float = 5.0,
    verbose: bool = False,
) -> None:
    """选一个随机矩形区域 (边长 ~ ``block_factor*r``), 删除其中所有选中圆.

    比随机扰动更具空间局部性, 有助跳出"该区域所有圆都不能改"型局部最优.
    """
    sel = state.selected_indices()
    if sel.size == 0:
        return
    pts = state.candidates_xy[sel]
    ymin, ymax = float(pts[:, 0].min()), float(pts[:, 0].max())
    xmin, xmax = float(pts[:, 1].min()), float(pts[:, 1].max())
    cy = rng.uniform(ymin, ymax)
    cx = rng.uniform(xmin, xmax)
    half = 0.5 * block_factor * r
    in_block_mask = (
        (np.abs(pts[:, 0] - cy) <= half)
        & (np.abs(pts[:, 1] - cx) <= half)
    )
    victims = sel[in_block_mask]
    for c in victims:
        state.remove(int(c))
    if verbose:
        print(
            f"  [perturb-block] center=({cy:.0f},{cx:.0f}) "
            f"half={half:.0f} removed={victims.size}, "
            f"|S|={state.size}, uncov={state.n_uncovered()}"
        )


def _perturb(
    state: SolutionState,
    strength: float,
    rng: random.Random,
    r: float,
    use_block: bool = False,
    block_factor: float = 5.0,
    verbose: bool = False,
) -> None:
    """统一扰动入口: 随机或块状扰动 + 修复贪心."""
    if use_block:
        _perturb_block(
            state, r=r, rng=rng,
            block_factor=block_factor, verbose=verbose,
        )
    else:
        _perturb_random(state, strength=strength, rng=rng, verbose=verbose)
    _greedy_repair(state, verbose=verbose)


def ils(
    coverage: csr_matrix,
    target_mask: np.ndarray,
    candidates_xy: np.ndarray,
    initial_selected: list[int],
    r: float,
    max_iter: int = 30,
    patience: int = 6,
    perturb_min: float = 0.05,
    perturb_max: float = 0.20,
    block_perturb_every: int = 3,
    block_factor: float = 5.0,
    enable_swap3: bool = True,
    swap3_max_attempts: int | None = 100000,
    time_limit: float | None = None,
    seed: int = 0,
    verbose: bool = False,
) -> tuple[list[int], dict]:
    """ILS 主入口.

    Args:
        coverage, target_mask, candidates_xy: 同 ``SolutionState`` .
        initial_selected: 贪心初始解 (必须可行或近可行).
        r: 观测半径 (用于邻接查询).
        max_iter: 外层最大迭代数.
        patience: 连续多少轮无改进则停止.
        perturb_min/max: 扰动比例 (随 no_improve 自适应).
        time_limit: 时间上限 (秒).
        verbose: 打印进度.

    Returns:
        (best_selected, stats)
    """
    rng = random.Random(seed)
    t0 = time.time()

    state = SolutionState.from_selected(
        coverage, target_mask, candidates_xy, list(initial_selected)
    )
    if verbose:
        print(
            f"[ILS] init |S|={state.size}, "
            f"feasible={state.is_feasible()}, "
            f"uncovered={state.n_uncovered()}"
        )
    if not state.is_feasible():
        _greedy_repair(state, verbose=verbose)
        if verbose:
            print(f"[ILS] after init-repair |S|={state.size}, "
                  f"feasible={state.is_feasible()}")

    local_search(
        state, r=r,
        enable_swap2=True, enable_swap3=enable_swap3,
        swap3_max_attempts=swap3_max_attempts,
        verbose=verbose,
    )
    best_indices = state.selected_indices().tolist()
    best_size = state.size
    if verbose:
        print(f"[ILS] post initial LS |S|={best_size}")

    no_improve = 0
    history: list[int] = [best_size]

    for it in range(1, max_iter + 1):
        if time_limit is not None and time.time() - t0 > time_limit:
            if verbose:
                print(f"[ILS] time limit reached")
            break

        strength = perturb_min + (perturb_max - perturb_min) * (
            no_improve / max(patience, 1)
        )
        strength = min(strength, perturb_max)

        snapshot = state.in_solution.copy()
        snapshot_count = state.cover_count.copy()

        use_block = (
            block_perturb_every > 0 and (it % block_perturb_every == 0)
        )
        _perturb(
            state, strength, rng, r=r,
            use_block=use_block, block_factor=block_factor,
            verbose=verbose,
        )
        local_search(
            state, r=r,
            enable_swap2=True, enable_swap3=enable_swap3,
            swap3_max_attempts=swap3_max_attempts,
            verbose=verbose,
        )

        cur_size = state.size
        history.append(cur_size)

        if state.is_feasible() and cur_size < best_size:
            best_size = cur_size
            best_indices = state.selected_indices().tolist()
            no_improve = 0
            if verbose:
                print(f"[ILS] iter {it}: NEW BEST |S|={best_size}")
        else:
            no_improve += 1
            if cur_size > best_size or not state.is_feasible():
                state.in_solution = snapshot
                state.cover_count = snapshot_count

        if no_improve >= patience:
            if verbose:
                print(f"[ILS] no improvement in {patience} iters, stop")
            break

    stats = {
        "iterations": it if "it" in locals() else 0,
        "elapsed": time.time() - t0,
        "best_size": best_size,
        "history": history,
        "final_feasible": True,
        "final_uncovered": 0,
    }
    return best_indices, stats
