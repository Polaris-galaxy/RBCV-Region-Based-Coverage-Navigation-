"""地图 IO 与预处理。

约定:
    - 原始栅格: ``np.ndarray[H, W], dtype=bool``
      ``True`` 表示自由空间, ``False`` 表示障碍。
    - 坐标顺序统一使用 ``(row, col)`` (即 numpy 的 ``[y, x]``)。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt
from skimage.morphology import medial_axis


@dataclass
class GridMap:
    """预处理完成的地图容器。

    Attributes:
        free: ``[H, W]`` bool, 自由空间掩码。
        edt:  ``[H, W]`` float, 自由像素到最近障碍的欧氏距离 (像素)。
              障碍像素处 edt = 0。
        skeleton: ``[H, W]`` bool, 中轴 (medial axis) 掩码。
        skel_dist: ``[H, W]`` float, 中轴上每点的局部半径
                   (= edt 在该处的取值)。非骨架像素为 0。
    """

    free: np.ndarray
    edt: np.ndarray
    skeleton: np.ndarray
    skel_dist: np.ndarray

    @property
    def shape(self) -> tuple[int, int]:
        return self.free.shape  # type: ignore[return-value]

    @property
    def n_free(self) -> int:
        return int(self.free.sum())


def load_map(path: str | Path, free_threshold: int = 127) -> np.ndarray:
    """从 PNG/PGM 等灰度图载入二值地图。

    Args:
        path: 图像路径。
        free_threshold: 灰度大于该阈值视为自由空间。

    Returns:
        ``[H, W]`` bool 自由空间掩码。
    """
    import imageio.v3 as iio

    img = iio.imread(path)
    if img.ndim == 3:
        img = img[..., :3].mean(axis=-1)
    return (img > free_threshold).astype(bool)


@dataclass
class RosMapInfo:
    """ROS map_server YAML 内描述的地图元数据."""

    image_path: Path
    resolution: float
    origin: tuple[float, float, float]
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196
    negate: int = 0


def _parse_yaml_simple(path: str | Path) -> dict:
    """极简 YAML 解析 (仅支持 key: value / [a, b, c] 行).

    避免引入 PyYAML 依赖. 不支持嵌套.
    """
    out: dict = {}
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            k = k.strip()
            v = v.strip()
            if v.startswith("[") and v.endswith("]"):
                items = [s.strip() for s in v[1:-1].split(",")]
                parsed = []
                for item in items:
                    try:
                        parsed.append(float(item))
                    except ValueError:
                        parsed.append(item)
                out[k] = parsed
            else:
                try:
                    out[k] = float(v)
                except ValueError:
                    out[k] = v
    return out


def _drop_border_touching_components(free: np.ndarray) -> np.ndarray:
    """剔除触及图像边界的自由连通域 (视为 "地图外" / 未扫描区域).

    ROS map_server 生成的 PGM 中, 地图扫描区外通常也是白色填充, 它们
    会与图像四个边相连; 真正室内的自由域被障碍包围, 不触及图像边界.
    """
    from scipy.ndimage import label as cc_label

    labels, n = cc_label(free)
    if n == 0:
        return free

    border_labels = set()
    for arr in (labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]):
        border_labels.update(int(x) for x in arr.tolist())
    border_labels.discard(0)
    if not border_labels:
        return free

    drop_mask = np.isin(labels, list(border_labels))
    return free & (~drop_mask)


def _keep_largest_component(free: np.ndarray) -> np.ndarray:
    """仅保留最大的 4-连通自由域."""
    from scipy.ndimage import label as cc_label

    labels, n = cc_label(free)
    if n <= 1:
        return free
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    keep_label = int(np.argmax(sizes))
    return labels == keep_label


def _resolve_ros_map_image_path(
    yaml_path: Path,
    cfg: dict,
    image_override: str | Path | None,
) -> Path:
    """根据 YAML + 可选覆盖解析 PGM 路径."""
    yaml_path = Path(yaml_path)
    if image_override is not None:
        return Path(image_override)
    img_field = str(cfg.get("image", ""))
    cand = Path(img_field)
    if cand.is_absolute() and cand.exists():
        return cand
    local = yaml_path.with_suffix(".pgm")
    if local.exists():
        return local
    local = yaml_path.parent / cand.name
    if local.exists():
        return local
    raise FileNotFoundError(
        f"PGM not found; tried: {cand}, {yaml_path.with_suffix('.pgm')}, "
        f"{yaml_path.parent / cand.name}"
    )


def _load_ros_gray_image(image_path: Path) -> np.ndarray:
    import imageio.v3 as iio

    img = iio.imread(image_path)
    if img.ndim == 3:
        img = img[..., :3].mean(axis=-1).astype(np.uint8)
    else:
        img = img.astype(np.uint8)
    return img


def _probability_grid_and_info(
    yaml_path: str | Path,
    image_override: str | Path | None = None,
) -> tuple[np.ndarray, RosMapInfo]:
    """读取 YAML + PGM，返回占用概率网格 ``g`` 与 ``RosMapInfo``.

    ``g`` 定义与 ``load_ros_map`` 一致: ``g = (255-p)/255`` (考虑 negate)。
    """
    yaml_path = Path(yaml_path)
    cfg = _parse_yaml_simple(yaml_path)
    image_path = _resolve_ros_map_image_path(yaml_path, cfg, image_override)

    img = _load_ros_gray_image(image_path)

    occ = float(cfg.get("occupied_thresh", 0.65))
    free_t = float(cfg.get("free_thresh", 0.196))
    res = float(cfg.get("resolution", 0.05))
    origin_field = cfg.get("origin", [0.0, 0.0, 0.0])
    if isinstance(origin_field, list) and len(origin_field) >= 3:
        origin = (
            float(origin_field[0]),
            float(origin_field[1]),
            float(origin_field[2]),
        )
    else:
        origin = (0.0, 0.0, 0.0)
    negate = int(cfg.get("negate", 0))

    p = img.astype(np.float32)
    if negate:
        p = 255.0 - p
    g = (255.0 - p) / 255.0

    info = RosMapInfo(
        image_path=image_path,
        resolution=res,
        origin=origin,
        occupied_thresh=occ,
        free_thresh=free_t,
        negate=negate,
    )
    return g.astype(np.float32), info


def load_ros_map(
    yaml_path: str | Path,
    image_override: str | Path | None = None,
    drop_border_components: bool = True,
    keep_largest_component: bool = True,
    extra_free_threshold: int | None = None,
    apply_clean_map: bool = True,
    clean_map_verbose: bool = False,
    clean_map_options: dict | None = None,
) -> tuple[np.ndarray, RosMapInfo]:
    """加载 ROS map_server 风格的 (.yaml + .pgm) 地图.

    像素阈值含义遵循 map_server 约定:
        ``g = (255 - p) / 255``      [p 为 PGM 像素值]
        ``g >= occupied_thresh``     -> 障碍
        ``g <= free_thresh``         -> 自由
        其它                         -> 未知 (此处保守视为障碍)

    Args:
        yaml_path: ``.yaml`` 路径.
        image_override: 若 YAML 中 ``image`` 字段为绝对路径但本机不存在,
            可手动指定本地 PGM 路径.
        keep_largest_component: 是否在 **地图清洗之后** 仅保留最大连通自由域.
            若 ``apply_clean_map=True``, 此项在 :func:`clean_map` **之后**
            执行; 清洗前不会裁剪连通域, 以便识别封闭中空区域.
        extra_free_threshold: 比 ``free_thresh`` 更紧的额外阈值 (像素值,
            0~255). 例如设为 240 后, 仅 240~255 视为真正自由. None=不启用.
        apply_clean_map: 是否对二值 free 做 :func:`clean_map`（障碍 disk closing
            弥缝、抹黑封闭区、轻度 free opening）. **默认 True**, 与当前脚本
            工具链一致; 需原始栅格时传 ``False``.
        clean_map_verbose: 为 True 时打印 :func:`clean_map` 的中文进度.
        clean_map_options: 覆盖传给 :func:`clean_map` 的参数
            (例 ``{"gap_seal_px": 7}``). 默认 ``gap_seal_px=5``,
            ``free_smooth_iter=1``, ``obstacle_smooth_iter=0``.

    Returns:
        (free_mask, info)  free_mask 为 ``[H, W]`` bool, ``info`` 含分辨率等.
    """
    yaml_path = Path(yaml_path)
    g, info = _probability_grid_and_info(yaml_path, image_override)
    free_t = info.free_thresh
    free = g <= free_t

    img_uint = _load_ros_gray_image(Path(info.image_path))
    if extra_free_threshold is not None:
        free = free & (img_uint >= extra_free_threshold)

    if drop_border_components:
        free = _drop_border_touching_components(free)

    if apply_clean_map:
        if clean_map_verbose:
            print(
                f"[load_ros_map] 清洗前可走像素 = {int(free.sum())} "
                f"（来自 {info.image_path}）"
            )
        opts: dict = {
            "fill_enclosed": True,
            "gap_seal_px": 5,
            "free_smooth_iter": 1,
            "obstacle_smooth_iter": 0,
            "verbose": clean_map_verbose,
        }
        if clean_map_options:
            opts.update(clean_map_options)
        free = clean_map(free, **opts)

    if keep_largest_component:
        free = _keep_largest_component(free)

    return free.astype(bool), info


def load_ros_map_occupancy(
    yaml_path: str | Path,
    image_override: str | Path | None = None,
) -> tuple[np.ndarray, RosMapInfo]:
    """加载 ROS ``nav_msgs/OccupancyGrid`` 语义的三态占用矩阵（不做连通域裁剪）。

    Returns:
        ``occupancy``: ``[H, W]`` ``int8``，取值 ``-1`` 未知、``0`` 自由、``100`` 占据，
        与常见 ``map_server`` 输出一致（介于 ``free_thresh`` 与 ``occupied_thresh``
        之间的概率栅格标为未知）。

        ``info``: 地图元数据（分辨率、原点、阈值）。

    Note:
        若需在规划中使用与 :func:`load_ros_map` 相同的「去边界 / 最大连通域」预处理，
        请继续在布尔 ``free`` 掩码上操作；本函数面向仿真 / 可视化 / 对外发布原始栅格。
    """
    g, info = _probability_grid_and_info(yaml_path, image_override)
    out = np.full(g.shape, -1, dtype=np.int8)
    out[g <= info.free_thresh] = 0
    out[g >= info.occupied_thresh] = 100
    return out, info


def meters_to_pixels(meters: float, resolution: float) -> float:
    return meters / resolution


def clean_map(
    free: np.ndarray,
    fill_enclosed: bool = True,
    gap_seal_px: int = 5,
    free_smooth_iter: int = 1,
    obstacle_smooth_iter: int = 0,
    fill_isolated_below_px: int | None = None,
    seal_gaps_px: int | None = None,
    smooth_iter: int | None = None,
    smooth_px: int | None = None,
    extra_smooth_iter: int | None = None,
    fill_isolated_free_below_px: int | None = None,
    verbose: bool = False,
) -> np.ndarray:
    """对二值自由空间做"先识别封闭区, 再轻度平滑"的图像级预处理.

    动机:
        ROS map_server 出来的 PGM 经常出现两类瑕疵:

        1. 一段几乎闭合的墙体, 因为扫描噪声断了几个像素的小缝, 让本应
           是 "封闭中空" 的内部 free 区与外部 free 通过缝隙连通; 直接做
           ``keep_largest_component`` 把不出来.
        2. 墙体表面有几个像素的毛刺, 让 medial axis / candidate 在那里
           反复挤压, 检测点重叠.

    流程 (严格按你要的顺序):
        1. **闭合识别**: 在障碍 ``~free`` 上用 ``disk(gap_seal_px)`` 圆形
           结构元素做一次 closing, 把所有宽度 ≤ ``2*gap_seal_px`` 像素
           的缝隙暂时堵上, 让 "几乎闭合" 的轮廓视为真闭合;
        2. **抹黑**: 在闭合后的图上找自由连通域, 凡是 *不与主自由域相连*
           的, 一律把它们的 *原始* 像素位置在 ``free`` 上抹黑成障碍.
           注意此处只读 closing 后的图做识别, 不直接把 closing 后的 free
           当输出, 因此真实窄通道不会被吃掉.
        3. **轻度平滑** (``free_smooth_iter`` 次, 默认 1): 在 free 上做
           opening, 抹平 1~2 个像素的边缘噪声; 不会扩张 free, 因此封闭
           轮廓不会被打通.
        4. **再次抹黑**: 平滑可能切出新的孤岛, 全部填实.
        5. **可选障碍光滑** (``obstacle_smooth_iter`` 次, **默认 0**):
           在墙体上做 opening, 把墙的毛刺磨平; 默认关闭, 避免上一版本
           出现的"薄墙被磨断"问题, 仅在确认墙体足够厚时再开.

    Args:
        free: ``[H, W]`` bool, ``True`` = 自由空间.
        fill_enclosed: 是否抹黑封闭中空图形, 默认 True.
        gap_seal_px: 闭合识别用的圆形结构元素 *半径* (像素), 缝隙弥合的
            最大宽度约为 ``2 * gap_seal_px``. 越大越激进, 太大会把真窄
            通道也认成封闭轮廓的一部分而吞掉; 默认 5 时, 0.05 m/px 分
            辨率下能弥合 ~50 cm 以内的扫描断口.
        free_smooth_iter: free 上 opening 次数 (轻度边缘平滑, 默认 1).
            想保留更多原图细节就设 0.
        obstacle_smooth_iter: 障碍 opening 次数 (墙体毛刺平滑, 默认 0).
            **会让薄墙变薄, 谨慎使用**.
        fill_isolated_below_px: 兼容旧参数, 仅在你不想用 ``fill_enclosed``
            而希望按面积阈值过滤时使用.
        seal_gaps_px / smooth_iter / smooth_px / extra_smooth_iter /
        fill_isolated_free_below_px: 上一版/上上版的兼容名.
        verbose: 中文进度.

    Returns:
        cleaned ``[H, W]`` bool 自由空间.
    """
    from scipy.ndimage import (
        binary_closing,
        binary_opening,
        generate_binary_structure,
        label as cc_label,
    )
    from skimage.morphology import disk

    if seal_gaps_px is not None:
        gap_seal_px = int(seal_gaps_px)
    if smooth_iter is not None:
        free_smooth_iter = int(smooth_iter)
    if smooth_px is not None:
        free_smooth_iter = int(smooth_px)
    if extra_smooth_iter is not None:
        obstacle_smooth_iter = int(extra_smooth_iter)
    if fill_isolated_free_below_px is not None and fill_isolated_below_px is None:
        fill_isolated_below_px = int(fill_isolated_free_below_px)

    if free.dtype != bool:
        free = free.astype(bool)

    out = free.copy()
    struct = generate_binary_structure(2, 1)
    before_free = int(out.sum())

    def _identify_non_main_components(mask: np.ndarray) -> tuple[np.ndarray, int, int]:
        """返回 (non_main_mask, n_dropped, dropped_px) — 全为 False 时表示只有主域."""
        labels, n_comp = cc_label(mask)
        if n_comp <= 1:
            return np.zeros_like(mask, dtype=bool), 0, 0
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        keep_label = int(np.argmax(sizes))
        non_main = (labels > 0) & (labels != keep_label)
        return non_main, n_comp - 1, int(non_main.sum())

    if fill_enclosed:
        if gap_seal_px and gap_seal_px > 0:
            obs = ~out
            kernel = disk(int(gap_seal_px)).astype(bool)
            obs_sealed = binary_closing(obs, structure=kernel)
            free_for_label = ~obs_sealed
            if verbose:
                print(
                    f"[地图清洗] 1) 障碍 closing(disk({gap_seal_px}))：暂时弥合 "
                    f"≤{2 * gap_seal_px} 像素宽的小缝隙以识别封闭曲线"
                )
        else:
            free_for_label = out

        non_main, n_dropped, dropped_px = _identify_non_main_components(
            free_for_label
        )
        if n_dropped > 0:
            keep_mask = ~non_main
            out = out & keep_mask
            if verbose:
                print(
                    f"[地图清洗] 2) 抹黑 {n_dropped} 个封闭中空区域"
                    f"（共 {dropped_px} 像素，原图位置直接置障）"
                )
        elif verbose:
            print("[地图清洗] 2) 未发现封闭中空区域，跳过抹黑步骤")

    if free_smooth_iter and free_smooth_iter > 0:
        prev = int(out.sum())
        out = binary_opening(
            out, structure=struct, iterations=int(free_smooth_iter)
        )
        if verbose:
            print(
                f"[地图清洗] 3) 自由域开运算 {free_smooth_iter} 次（轻度平滑）："
                f"可走像素 {prev} → {int(out.sum())}"
            )

        if fill_enclosed:
            non_main, n_dropped, dropped_px = _identify_non_main_components(out)
            if n_dropped > 0:
                out = out & ~non_main
                if verbose:
                    print(
                        f"[地图清洗] 4) 平滑后再次抹黑 {n_dropped} 个新孤岛"
                        f"（{dropped_px} 像素）"
                    )

    if obstacle_smooth_iter and obstacle_smooth_iter > 0:
        obs = ~out
        prev_obs = int(obs.sum())
        obs = binary_opening(
            obs, structure=struct, iterations=int(obstacle_smooth_iter)
        )
        out = ~obs
        if verbose:
            print(
                f"[地图清洗] 5) 障碍开运算 {obstacle_smooth_iter} 次（墙体光滑）："
                f"障碍像素 {prev_obs} → {int(obs.sum())}（注意：会让薄墙变薄）"
            )
        if fill_enclosed:
            non_main, n_dropped, dropped_px = _identify_non_main_components(out)
            if n_dropped > 0:
                out = out & ~non_main
                if verbose:
                    print(
                        f"[地图清洗] 5.1) 墙体光滑后再次抹黑 {n_dropped} 个新孤岛"
                        f"（{dropped_px} 像素）"
                    )

    if fill_isolated_below_px is not None and fill_isolated_below_px > 0:
        labels, n_comp = cc_label(out)
        if n_comp > 0:
            sizes = np.bincount(labels.ravel())
            sizes[0] = 0
            keep = sizes >= int(fill_isolated_below_px)
            keep_mask = keep[labels]
            removed_px = int((out & ~keep_mask).sum())
            removed_cnt = int(np.sum((sizes > 0) & ~keep))
            out = out & keep_mask
            if verbose and removed_cnt > 0:
                print(
                    f"[地图清洗] 6) 额外阈值过滤：填实 {removed_cnt} 个 "
                    f"<{fill_isolated_below_px}px 的零碎自由片段"
                    f"（{removed_px} 像素）"
                )

    if verbose:
        print(
            f"[地图清洗] 完成：可走像素 {before_free} → {int(out.sum())} "
            f"（变化 {int(out.sum()) - before_free:+d}）"
        )

    return out.astype(bool)


def preprocess(free: np.ndarray, min_corridor: float = 1.0) -> GridMap:
    """对二值地图做距离变换 + 中轴提取。

    Args:
        free: ``[H, W]`` bool 自由空间掩码。
        min_corridor: 最小通道宽度 (像素)。
            ``edt`` 小于 ``min_corridor/2`` 的像素将被视为不可通行,
            可消除栅格化产生的单像素细线。

    Returns:
        ``GridMap``.
    """
    if free.dtype != bool:
        free = free.astype(bool)

    edt = distance_transform_edt(free).astype(np.float32)

    if min_corridor > 1.0:
        free = free & (edt >= min_corridor / 2.0)
        edt = distance_transform_edt(free).astype(np.float32)

    skeleton, dist_on_skel = medial_axis(free, return_distance=True)
    skel_dist = np.where(skeleton, dist_on_skel.astype(np.float32), 0.0)

    return GridMap(
        free=free.astype(bool),
        edt=edt,
        skeleton=skeleton.astype(bool),
        skel_dist=skel_dist,
    )
