# 操作指南：地图分区 + 圆盘覆盖（RBCV）

> **RBCV** = Region-Based Coverage Navigation。  
> 目标：把 **`src/map_tools/maps/`** 里的栅格地图手动划分长方形分区，再用 **`coverage_planner`** 对每个分区做半径 **2.5 m** 的检测圆覆盖。**离线步骤全程纯 Python，不需要 ROS。**

---

## 0. 前置环境（一次性）

在 **仓库根目录** 下的 `src/` 安装依赖（路径按你本机克隆位置替换 `<仓库根>`）：

```powershell
cd <仓库根>\src
pip install -r requirements.txt
```

示例（文件夹名为 `RBCV-Region-Based-Coverage-Navigation-` 时）：

```powershell
cd D:\Galaxy\其他\桌面\RBCV-Region-Based-Coverage-Navigation-\src
pip install -r requirements.txt
```

依赖：`numpy` `scipy` `scikit-image` `imageio` `matplotlib` `pytest`。

> 后续命令均假设当前目录为 **`src/coverage_planner`**（执行 **`scripts/`** 下工具时的工作目录）。

---

## 1. 准备地图

地图统一放在 **`src/map_tools/maps/`**：

```
map_tools/maps/
├── map7.yaml + map7.pgm           # ROS map_server 风格（YAML 中 image 用相对路径）
├── map8.yaml + map8.pgm
└── regions_map7.example.json      # 示例分区
```

新地图：把 `*.yaml` + `*.pgm` 放进该目录，且 `yaml` 里 **`image:`** 字段使用 **相对文件名**（如 `image: map9.pgm`）。

> 想先看地图边界范围（米）：
>
> ```powershell
> py scripts\preview_real_map.py ..\map_tools\maps\map7.yaml
> ```
>
> 终端会打印 `shape`、`resolution`，结合 `yaml` 里的 `origin` 即可推算 x/y 范围。

---

## 2. 划分长方形区域

以下两种方式任选其一，**都生成同一种 JSON**。

### 方式 A：交互式拖框（推荐）

```powershell
py scripts\draw_regions.py ..\map_tools\maps\map7.yaml ..\map_tools\maps\my_regions.json
```

操作（窗口标题也有提示）：

| 操作 | 含义 |
|------|------|
| 鼠标左键拖动 | 画一个长方形分区（自动命名 `R0/R1/...`） |
| `n` | 终端里输入下一个矩形的字符串 id（如 `room_A`） |
| `u` | 撤销最后一个矩形 |
| `r` | 清空全部 |
| **`s`** | **保存** 到 `my_regions.json` |
| `q` | 关闭窗口 |

显示坐标轴单位为 **米**，已用 `RosMapInfo.origin + resolution` 对齐 ROS 约定。

### 方式 B：直接编辑 JSON

复制 `regions_map7.example.json` 改名（如 `my_regions.json`），按需要增删矩形：

```json
{
  "frame_id": "map",
  "regions": [
    {"id": "room_A",  "xmin": -48.0, "ymin": -50.0, "xmax": -35.0, "ymax": -38.0},
    {"id": "corridor","xmin": -35.0, "ymin": -45.0, "xmax": -22.0, "ymax": -42.0}
  ]
}
```

字段：

- **`frame_id`**：通常 `map`，与发布占用栅格的帧一致；
- **`xmin/ymin/xmax/ymax`**：**米**，`map` 坐标系；
- **`id`**：任意字符串，仅作标识；
- 矩形可重叠：**列表中靠前者优先**（`region_io.rasterize_regions` 已实现）。

---

## 3. 验证分区对齐

```powershell
py scripts\preview_regions.py ..\map_tools\maps\map7.yaml ..\map_tools\maps\my_regions.json
```

终端会列出每个分区的：

| 字段 | 含义 |
|------|------|
| `矩形内像素` | 矩形覆盖到的栅格数 |
| `可走∩矩形` | 与自由空间相交的栅格数（**= 0 通常说明矩形落在障碍 / 地图外**） |

弹出窗口：左图叠加矩形边框，右图用不同颜色显示分区 ID（自动保存到 `scripts/分区叠图预览.png`）。

---

## 4. 对每个分区做圆盘覆盖（半径 2.5 m）

```powershell
py scripts\demo_regions.py ..\map_tools\maps\map7.yaml 2.5 ..\map_tools\maps\my_regions.json
```

执行流程（每个分区 **独立** 完成）：

1. **预处理**：在 region 子图上算 EDT、中轴；
2. **形状分类**（自动）：

   | `kind` | 触发条件 | 候选策略 |
   |--------|----------|----------|
   | `open`   | 矩形度高 + 自由半径足够 | **仅六边形**（最快） |
   | `narrow` | 平均自由半径 < 0.6 r | **中轴 + 反射顶点**，关闭六边形 |
   | `mixed`  | 介于之间 | 三类全开 |
   | `tiny`   | 过小 | 少量随机点 + 关闭 ILS |

3. **求解**：贪心 Set Cover（带 `overlap_tiebreak` 减少边缘/交叉口密集叠加） → ILS 精修。

输出：

- 终端：每个分区一行（类型、可走像素、形状度量、圆盘数、覆盖率等，中文日志）；
- 图片：`scripts/分区覆盖结果.png`（按分区分色，每个圆 = 一个 `r = 2.5 m` 检测点）。

---

## 5. 程序内调用（接入下游 Python）

```python
from coverage_planner import (
    load_ros_map, load_regions_json, rasterize_regions,
    PlannerConfig, plan_partitions,
)
from coverage_planner.map_io import meters_to_pixels

# 路径与示例脚本一致：工作目录为 src/coverage_planner 时用 ../map_tools/maps/
# load_ros_map 默认已做 clean_map（闭合弥缝、抹黑封闭中空区等）；需原始栅格时传 apply_clean_map=False
free, info = load_ros_map("../map_tools/maps/map7.yaml")
spec = load_regions_json("../map_tools/maps/my_regions.json")
labels = rasterize_regions(spec, info, free.shape)

partition = plan_partitions(
    free_full=free,
    labels=labels,
    r_px=meters_to_pixels(2.5, info.resolution),
    base_config=PlannerConfig(overlap_tiebreak=True, ils_max_iter=40),
    verbose=True,
)

print("总圆数:", partition.total_circles)
pts_px = partition.all_points()      # [N, 2] 像素 (row, col)
```

像素 → 米（与 ROS 一致）：

```python
ox, oy, _ = info.origin
res = info.resolution
mx = ox + (pts_px[:, 1] + 0.5) * res
my = oy + (pts_px[:, 0] + 0.5) * res
```

`partition.region_plans[i]` 内含 `region_id / shape / config / result`，可单独取每个分区的结果。

---

## 6. 调参与排错

| 现象 | 处理 |
|------|------|
| 某分区报 `px_in_rect∩free = 0` | 矩形落在障碍/地图外，回到 **第 2 步** 重画或调坐标 |
| 方正区域结果还偏密 | 把 `PlannerConfig.overlap_tiebreak=True`（默认开）；或减小 `ils_max_iter` 缩短运行 |
| 窄通道断开/不足 | `PlannerConfig(min_corridor=2.0)` 调大；或减小 `alpha`（`candidates._step_length`） |
| 两个相邻矩形交界出现密集圆 | 把交界处归并为一个矩形；或在 `coverage_regions_ros` 里通过 region 列表顺序控制归属 |
| 改了 yaml 后规划坐标不对 | 检查 `yaml` 中 **`origin`** 与实际地图原点一致，`resolution` 单位为米/像素 |

---

## 7. ROS1（后续步骤，可暂时忽略）

`coverage_regions_ros` 与 `map_tools` 已写好 catkin 包结构，未来在 Ubuntu Noetic 工作空间里：

```bash
catkin build && source devel/setup.bash
roslaunch map_tools publish_static_map.launch
roslaunch coverage_regions_ros publish_partition_labels.launch \
    regions_json:=$(rospack find map_tools)/maps/my_regions.json
```

会发布两个话题：

- `/map`（`nav_msgs/OccupancyGrid`）：原栅格地图
- `/region_partition_labels`（`OccupancyGrid`）：分区标签栅格（`-1` = 区外，`1..K` = 第 k 个分区）

下游 Python 节点订阅 `/region_partition_labels` 即可与本指南的离线流程对齐。

---

## 文件速查（相对仓库根）

| 用途 | 路径 |
|------|------|
| 地图 YAML/PGM、分区 JSON | `src/map_tools/maps/` |
| 分区解析（Python 模块） | `src/coverage_planner/coverage_planner/region_io.py` |
| 分区自适应规划（Python 模块） | `src/coverage_planner/coverage_planner/region_planner.py` |
| 交互划分脚本 | `src/coverage_planner/scripts/draw_regions.py` |
| 分区预览脚本 | `src/coverage_planner/scripts/preview_regions.py` |
| 端到端覆盖脚本 | `src/coverage_planner/scripts/demo_regions.py` |

更细的算法说明见 `src/coverage_planner/docs/ALGORITHM.md`。
