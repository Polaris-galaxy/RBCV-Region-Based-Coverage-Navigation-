# 操作指南：地图分区 + 圆盘覆盖（RBCV）

**RBCV** = Region-Based Coverage Navigation。  
目标：在 **`src/map_tools/maps/`** 中为栅格地图划分长方形分区，再在 **`src/rbcv_disk_coverage/`** 下用 **`coverage_planner`** 对每个分区做半径 **2.5 m** 的检测圆覆盖。**离线规划全程纯 Python，不需要 ROS。**

---

## 快速索引

| 你想做… | 跳到 |
|--------|------|
| 装依赖、画分区、跑覆盖 | [第 0–4 节](#0-前置环境一次性) |
| 在代码里调用 `plan_partitions` | [第 5 节](#5-程序内调用接入下游-python) |
| 环境变量、排错、`RBCV_BEST` | [第 6 节](#6-调参与排错) |
| 语义探索 + rosbag | [第 7 节](#7-语义探索回路explorationrosbag--视觉--导航闭环) |
| ROS1 发布 `/map` 与分区栅格；两阶段 launch（分区→survey / 语义栈） | [第 8 节](#8-ros1-后续步骤可暂时忽略) |
| **从地图到闭合路径（总览表）** | 快速索引下一节 **「端到端总览」** |

---

## 端到端总览：从地图到语义路径与代价

下面把你心里的顺序与仓库里的**真实产物**对齐（括号内为文档章节或命令入口）。

| 步骤 | 你做的 | 在本仓库里对应什么 | 产出 / 备注 |
|------|--------|-------------------|-------------|
| 1 | 导入地图 | [§1 准备地图](#1-准备地图)；ROS 场景下再 [§8.1](#81-一键启动推荐rbcv_bringup) 发布 `/map` | `map_tools/maps/*.yaml` + `*.pgm` |
| 2 | 划分区域 | [§2 长方形分区](#2-划分长方形区域)；可选 [§3 验证](#3-验证分区对齐) | `my_regions.json`（米、`map` 系） |
| 3 | 生成检测框 | [§4 `demo_regions.py`](#4-圆盘覆盖demo_regionspy) 对**每个分区**算圆盘覆盖点；或 **ROS** [§8.4 阶段一](#84-两阶段-launch分区--检测框-json语义探索--计划-json) 写出 `survey_zones.json` | **`survey_zones.json`**：每项含 `zone_id`、`bbox_xyxy`/`polygon_xy`、**`coverage.sites_m`**（覆盖站位） |
| 4 | 初始区域权重 | 在 **`survey_zones.json`** 里给部分 zone 写 **`base_weight_hint`**；或在跑语义栈时用 **`--priors-json`**（按 `zone_id` 指定 `InitialRegionWeight`），见 `exploration/README.md` | 与后续探测计数在 `aggregation` 里融合 |
| 5 | 用 bag 给检测框赋权 | [§7](#7-语义探索回路explorationrosbag--视觉--导航闭环)：`run_semantic_stack.py`（或阶段二 launch）传入 **`--bag`、`--topic /odom`、`--detections`**；离线用 **`rosbags`** 插值机器人 **2D** 位置，仅在 **coverage 内**的视觉观测参与计数，再得到 **框级权重** | **`zone_weights`**（及 `per_zone_class_counts`）。无 bag 时只做自检，权重主要来自先验与空计数 |
| 6 | 生成「方形」区域 | **`exploration`** 在工作区外包络上划 **均匀矩形格**（可调 `--cell-size`），不是再画一批检测框；格内聚合各检测框权重 + 可选矩形先验 | **`rect_weights`**，格心即路径顶点候选 |
| 7 | 代价地图 | **`exploration/costmap.py`**：由 **`rect_weights`** 与 `PathPlannerParams` 得到边上「代价−（权重）奖励」，用于图搜索；这是 **语义层** 权图，**不是**直接替换 `move_base` 的 `costmap_2d`；若要接 Nav，需自行把权重同步到 costmap 层 | 计划 JSON 里不单独落盘占据栅格，算法在栈内完成 |
| 8 | 最终路径 | 同上命令输出 **`waypoints_xy`**：**闭合**回路（起点=终点），NN + 2-opt；可选 C++ `rbcv_plan_tour` 加速，见 `exploration_core_cpp/README.md` | **`out_semantic_plan.json` / `rbcv_semantic_plan.json`**；接 **move_base** 时仍要叠 **`/map` 占据** 做碰撞约束 |

**一句话顺序：** 地图 → `regions.json` → 圆盘覆盖 → **`survey_zones.json`** →（写先验）→ **`run_semantic_stack`**（+ 可选 bag + `detections.jsonl`）→ **矩形格权重 + 语义代价 + `waypoints_xy`**。

若你原文「权重复制」是指 **把某一区域的权重拷贝到另一区域**，当前脚本没有专用开关；需要可在 JSON 里手工复制 `base_weight_hint` / 后处理 `zone_weights`，或为多个 `zone_id` 配同一条 `--priors-json` 记录。

---

## 0. 前置环境（一次性）

在仓库 **`src/`** 下安装依赖（把 `<仓库根>` 换成本机路径）：

**Linux / macOS：**

```bash
cd <仓库根>/src
python3 -m pip install -r requirements.txt
```

**Windows（PowerShell）：**

```powershell
cd <仓库根>\src
py -m pip install -r requirements.txt
```

依赖：`numpy` `scipy` `shapely` `rosbags` `scikit-image` `imageio` `matplotlib` `numba` `pytest`（`numba` 用于可见性 JIT 加速）。

**工作目录约定：** `scripts/demo_regions.py` 等命令的当前目录必须是 **`src/rbcv_disk_coverage`**，见 [第 4 节](#4-圆盘覆盖demo_regionspy)；其它包内 `scripts/` 同理。

---

## 1. 准备地图

地图放在 **`src/map_tools/maps/`**：

```
map_tools/maps/
├── map7.yaml + map7.pgm           # ROS map_server 风格（YAML 中 image 用相对路径）
├── map8.yaml + map8.pgm
├── regions_map7.example.json      # 示例分区
└── my_regions.json                # 自画/自编辑分区（示例仓库里可能有）
```

新地图：放入 `*.yaml` + `*.pgm`，且 YAML 里 **`image:`** 使用**相对文件名**（如 `image: map9.pgm`）。

查看地图栅格形状与分辨率（米/像素），在 **`src/rbcv_disk_coverage`** 下：

```bash
python3 scripts/preview_real_map.py ../map_tools/maps/map7.yaml
```

（Windows 可把 `python3` 换成 `py`。）终端会打印 `shape`、`resolution`，结合 YAML 里的 `origin` 可推算 x/y 范围。

---

## 2. 划分长方形区域

两种方式任选，**输出同一种 JSON**。请先 **`cd` 到 `src/rbcv_disk_coverage`**，与 [第 4 节](#4-圆盘覆盖demo_regionspy) 一致。

### 方式 A：交互式拖框（推荐）

```bash
python3 scripts/draw_regions.py ../map_tools/maps/map7.yaml ../map_tools/maps/my_regions.json
```

| 操作 | 含义 |
|------|------|
| 鼠标左键拖动 | 画长方形分区（自动命名 `R0/R1/...`） |
| `n` | 在终端输入下一个矩形的字符串 id（如 `room_A`） |
| `u` | 撤销最后一个矩形 |
| `r` | 清空全部 |
| **`s`** | **保存** 到目标 JSON |
| `q` | 关闭窗口 |

坐标轴单位为 **米**，与 `RosMapInfo.origin + resolution` 的 ROS 约定一致。

Windows 若中文乱码：脚本会尝试注册 `Microsoft YaHei/SimHei`；仍异常请确认系统存在 `C:\Windows\Fonts\msyh.ttc`。

### 方式 B：直接编辑 JSON

复制 `regions_map7.example.json` 改名后编辑：

```json
{
  "frame_id": "map",
  "regions": [
    {"id": "room_A",  "xmin": -48.0, "ymin": -50.0, "xmax": -35.0, "ymax": -38.0},
    {"id": "corridor","xmin": -35.0, "ymin": -45.0, "xmax": -22.0, "ymax": -42.0}
  ]
}
```

- **`frame_id`**：通常 `map`，与占用栅格帧一致。  
- **`xmin/ymin/xmax/ymax`**：**米**，`map` 坐标系。  
- **`id`**：任意字符串。  
- 矩形可重叠：**列表中靠前者优先**（`region_io.rasterize_regions`）。

---

## 3. 验证分区对齐

工作目录：**`src/rbcv_disk_coverage`**。

```bash
python3 scripts/preview_regions.py ../map_tools/maps/map7.yaml ../map_tools/maps/my_regions.json
```

终端列出每个分区的 `矩形内像素`、`可走∩矩形`（后者为 0 通常表示矩形落在障碍或地图外）。  
弹窗左图叠加矩形，右图按分区着色；预览图默认写到 `scripts/分区叠图预览.png`（运行时生成，已在仓库 `.gitignore`）。

---

## 4. 圆盘覆盖（`demo_regions.py`）

```bash
cd <仓库根>/src/rbcv_disk_coverage
python3 scripts/demo_regions.py ../map_tools/maps/map7.yaml 2.5 ../map_tools/maps/my_regions.json
```

**候选流水线（默认）：** **`RBCV_HEX_FIRST=1`**（默认开启）为「先六边形铺底 → 残余再补圆」；**`RBCV_HEX_FIRST=0`** 回到旧流水线。更细的 hex 相位搜索、大厅冻结、`RBCV_GLOBAL_HEX_FREEZE`、`RBCV_TRIM_MODE`（`local` / `ils` / `regreedy`）等见 **`rbcv_disk_coverage/docs/ALGORITHM.md`** 与 **`rbcv_disk_coverage/README.md`**。

**并行：** **`RBCV_PAR_BACKEND`** 取 `process`（默认，多进程）或 `thread`（多线程，常受 GIL 限制）。在运行 `demo_regions.py` 之前设置，例如 `export RBCV_PAR_BACKEND=process`。单分区时并行几乎不加速；**全局精修**在各区结束后仍串行。

**时间上限：** **`RBCV_REGION_ILS_TIME_LIMIT`**（各分区内精修，秒）、**`RBCV_TRIM_TIME_LIMIT`**（合并后整图精修）。脚本内另有默认上限（见 `demo_regions.py` 顶部常量）。

**其它常用环境变量：** **`RBCV_FAST=1`**（省时间可能降质量）、**`RBCV_N_RAYS`**、`COVERAGE_PLANNER_USE_NUMBA=0` 关闭 JIT、**`RBCV_HEX_FIRST_FULL_ILS=1`** 更强扰动精修。走廊划条须保证 YAML 分辨率传入（脚本已从地图读取）。

**产出：** 终端汇总 + **`scripts/分区覆盖结果.png`**（已 `.gitignore`）。**测试：** 在 `rbcv_disk_coverage` 下执行 `python3 -m pytest tests -q`。

---

## 5. 程序内调用（接入下游 Python）

```python
from coverage_planner import (
    load_ros_map, load_regions_json, rasterize_regions,
    PlannerConfig, plan_partitions,
)
from coverage_planner.map_io import meters_to_pixels

free, info = load_ros_map("../map_tools/maps/map7.yaml")
spec = load_regions_json("../map_tools/maps/my_regions.json")
labels = rasterize_regions(spec, info, free.shape)

partition = plan_partitions(
    free_full=free,
    labels=labels,
    r_px=meters_to_pixels(2.5, info.resolution),
    base_config=PlannerConfig(overlap_tiebreak=True, ils_max_iter=40),
    resolution_m_per_px=float(info.resolution),
    verbose=True,
)

print("总圆数:", partition.total_circles)
pts_px = partition.all_points()
```

像素 → 米（与 ROS 一致）：

```python
ox, oy, _ = info.origin
res = info.resolution
mx = ox + (pts_px[:, 1] + 0.5) * res
my = oy + (pts_px[:, 0] + 0.5) * res
```

`partition.region_plans[i]` 含 `region_id / shape / config / result`；**`shape.kind`** 含 `rect_room`、`corridor`、`narrow`、`mixed`、`tiny`；走廊划条摘要见 **`shape.corridor_plan_meta`**。

---

## 6. 调参与排错

### 6.1 一键「尽量最优」（更慢、更省圆）

工作目录仍为 **`src/rbcv_disk_coverage`**。

**bash：**

```bash
export RBCV_BEST=1
python3 scripts/demo_regions.py ../map_tools/maps/map7.yaml 2.5 ../map_tools/maps/my_regions.json
```

**PowerShell：**

```powershell
$env:RBCV_BEST="1"
py scripts\demo_regions.py ..\map_tools\maps\map7.yaml 2.5 ..\map_tools\maps\my_regions.json
```

常用覆盖项：`RBCV_TRIM_MODE`、`RBCV_SOFT_FACTOR`、`RBCV_TRIM_TIME_LIMIT` 等。

| 现象 | 处理 |
|------|------|
| `px_in_rect∩free = 0` | 矩形落在障碍/地图外，回到 [第 2 节](#2-划分长方形区域) |
| 方正区域仍偏密 | `overlap_tiebreak=True`（默认）；或减小 `ils_max_iter` |
| 窄通道断开 | `PlannerConfig(min_corridor=2.0)` 等，见包内 README |
| 相邻矩形交界圆过密 | 合并矩形或调整 `regions` 列表顺序 |
| 改 yaml 后坐标不对 | 检查 `origin`、`resolution` |

---

## 7. 语义探索回路（`exploration`：rosbag + 视觉 + 导航闭环）

**`src/exploration/`** 在检测框与覆盖站位上统计目标、给框赋权、矩形格权重与闭合探索路径；与 [第 2–4 节](#2-划分长方形区域) 的几何圆盘覆盖互补。

1. **`survey_zones.json`**（示例：`exploration/data/survey_zones.example.json`）  
2. **`detections.jsonl`**（示例：`exploration/data/detections.example.jsonl`）  
3. **ROS1** bag；**推荐**话题 **`/odom`**（**`nav_msgs/Odometry`**）。解析时仅用平面 **`x, y`** 与检测时间对齐；分区与检测坐标建议在 **`map`** 系并与 bag 位姿一致。可用 `python3 exploration/scripts/inspect_rosbag1.py <bag>` 查看话题。

在 **`src/`** 下生成计划 JSON：

```bash
cd <仓库根>/src
python3 exploration/scripts/run_semantic_stack.py \
  --survey exploration/data/survey_zones.example.json \
  --bag /path/to/record.bag --topic /odom \
  --detections exploration/data/detections.example.jsonl \
  --out exploration/data/out_semantic_plan.json
```

仅自检可省略 `--bag` / `--topic` / `--detections`。更多参数见 `python3 exploration/scripts/run_semantic_stack.py --help`。

目录索引：`src/WORKSPACE_LAYOUT.md`；模块说明：`src/exploration/README.md`。

---

## 8. ROS1（后续步骤，可暂时忽略）

### 8.1 一键启动（推荐）：`rbcv_bringup`

组合 launch 单独放在包 **`src/rbcv_bringup/launch/`**，一次拉起静态地图与分区标签（内部再包含 `map_tools`、`coverage_regions_ros` 的 launch）。

将本仓库 **`src/`** 放入你的 catkin 工作空间 `src/` 后编译并配置环境：

```bash
cd <你的 catkin 工作空间>
catkin build   # 或 catkin_make
source devel/setup.bash
```

默认地图 **`map7.yaml`**、分区 **`map_tools/maps/my_regions.json`**；可按需覆盖参数：

```bash
roslaunch rbcv_bringup rbcv_map_and_regions.launch
```

指定地图与分区文件：

```bash
roslaunch rbcv_bringup rbcv_map_and_regions.launch \
  map_yaml:=$(rospack find map_tools)/maps/map8.yaml \
  regions_json:=$(rospack find map_tools)/maps/regions_map7.example.json
```

可选：`frame_id:=map`（与 `publish_static_map.launch` 一致）。

### 8.2 分包单独启动

与旧文档等价：

```bash
roslaunch map_tools publish_static_map.launch
roslaunch coverage_regions_ros publish_partition_labels.launch \
  regions_json:=$(rospack find map_tools)/maps/my_regions.json
```

### 8.3 发布的话题

- **`/map`**（`nav_msgs/OccupancyGrid`）：栅格地图  
- **`/region_partition_labels`**（`nav_msgs/OccupancyGrid`）：分区标签（`-1` 区外，`1..K` 第 k 区）

下游 Python 节点订阅 `/region_partition_labels` 可与离线流程对齐。

### 8.4 两阶段 launch（分区 → 检测框 JSON；语义探索 → 计划 JSON）

| 文件 | 作用 |
|------|------|
| **`rbcv_stage1_partitions_to_survey.launch`** | 发布 **`/map`** 与 **`/region_partition_labels`**；运行 **`regions_to_survey_zones.py`**，按与 `demo_regions.py` 相同的环境变量做圆盘覆盖，并写出 **`survey_zones.json`**（各分区 `bbox_xyxy` + `coverage.sites_m`）。默认输出：`map_tools/maps/rbcv_survey_zones.json`。 |
| **`rbcv_stage2_semantic_explore.launch`** | 读入上一步的 survey（或其它 `survey_zones.json`），调用 **`rbcv_semantic_stack_cli.py`** 对接 `run_semantic_stack`。**`bag` 留空**为自检（无 rosbag）；若填写 **`bag`**，须同时填写 **`detections`**（`topic` 默认 `/odom`）。默认输出：`map_tools/maps/rbcv_semantic_plan.json`。 |

阶段一示例：

```bash
roslaunch rbcv_bringup rbcv_stage1_partitions_to_survey.launch \
  regions_json:=$(rospack find map_tools)/maps/my_regions.json \
  survey_out:=$(rospack find map_tools)/maps/rbcv_survey_zones.json
```

阶段二（仅自检）：

```bash
roslaunch rbcv_bringup rbcv_stage2_semantic_explore.launch
```

阶段二（含 bag + 检测；三参需齐）：

```bash
roslaunch rbcv_bringup rbcv_stage2_semantic_explore.launch \
  bag:=/path/to/record.bag \
  detections:=/path/to/detections.jsonl
```

（`detections` 请换成你机器上的绝对路径；`cell_size:=1.0` 等见 `run_semantic_stack.py --help`。）

---

## 文件速查（相对仓库根）

| 用途 | 路径 |
|------|------|
| 一键 ROS launch（组合） | `src/rbcv_bringup/launch/rbcv_map_and_regions.launch` |
| 两阶段：分区→survey JSON | `src/rbcv_bringup/launch/rbcv_stage1_partitions_to_survey.launch` |
| 两阶段：语义探索计划 JSON | `src/rbcv_bringup/launch/rbcv_stage2_semantic_explore.launch` |
| 地图 YAML/PGM、分区 JSON | `src/map_tools/maps/` |
| 分区解析 | `src/rbcv_disk_coverage/coverage_planner/region_io.py` |
| 分区规划 | `src/rbcv_disk_coverage/coverage_planner/region_planner.py`、`corridor_split.py` |
| 交互划分 / 预览 / 覆盖脚本 | `src/rbcv_disk_coverage/scripts/draw_regions.py` 等 |
| 语义探索 | `src/exploration/scripts/`、`src/exploration/README.md` |
| 算法细节 | `src/rbcv_disk_coverage/docs/ALGORITHM.md` |
