# `exploration/` — 语义调查与加权探索导航

与 **`rbcv_disk_coverage/`**（Python 包名 **`coverage_planner`**，几何**可见性圆盘覆盖**）分工如下：

| 层级 | 职责 |
|------|------|
| **几何覆盖** | 障碍栅格 + 长方形分区 → 圆盘观测点（`demo_regions.py` 等） |
| **本包** | 检测框 + **覆盖站位圆**、rosbag **机器人轨迹**、视觉 **JSONL** → 目标计数与去重 → **框权重** → 工作区 **均匀矩形格** → **代价/奖励** → **闭合路径** |

## 快速使用

1. **安装依赖**（在仓库根或 `src` 上级）：`pip install -r requirements.txt`（需含 `rosbags`、`shapely` 等）。
2. **进入 `src/`**，执行语义栈：
   - **有 bag + 检测**：`--survey`、`--bag`、`--topic /odom`、`--detections`、`--out` **五类参数都要齐**（`--bag` 与检测成对）。
   - **仅自检网格/路径**：省略 `--bag`、`--topic`、`--detections` 三者。
3. **示例命令**（见下节「一键栈」与 `scripts/run_semantic_stack.py` 顶部注释；**检视 bag 话题**：`py exploration/scripts/inspect_rosbag1.py <你的.bag>`）。
4. **ROS1 launch 两阶段**（需 catkin）：阶段一写出 `survey_zones.json`，阶段二写出语义计划 JSON；说明与默认 **`/odom`** 见 **`../rbcv_bringup/README.md`**，全流程见 **`../USAGE.md`** 第 7、8 节。
5. **输出**：`out_semantic_plan.json`（或 launch 指定的路径）中含 `zone_weights`、`rect_weights`、**`waypoints_xy`** 等；**`waypoints_xy`** 为 **2D** 闭合路径点，接 **move_base** 时仍需叠加 **`/map` 占据** 做避障。

## 物体记录策略（默认）

- 视觉若提供稳定 **`track_id`**：按 track 去重。  
- 否则：同一 `(zone_id, class)` 内对 `(x,y)` 做 **DBSCAN**（半径默认 **0.45 m**，与 `merge_radius_m` 一致）；簇数 = 唯一实例数。  
- **何时记入某框**：先用 rosbag 在检测时刻 **`t_s`** 上插值机器人位置；仅当机器人落在该框的 **coverage** 内时才接受该条检测（见下）。

## `survey_zones.json`（版本 2）

顶层 **`zones`** 数组。每项：

- `zone_id`、`bbox_xyxy` 或 `polygon_xy`、`label`、`base_weight_hint`（可选，且在 **`InitialRegionWeight`** 之外可与计数融合）。  
- **`coverage`**（可选）：  
  - `radius_m`：与 RBCV 规划半径一致为佳。  
  - `sites_m`：**圆盘中心列表**（米），一般从 `demo_regions` 结果抄入。  
  - 若省略 `sites_m` 或为空：**整个检测框多边形内**均视为「调查任务区」（机器人只要在框内即算到位）。

## 视觉侧 `detections.jsonl`

每行一个 JSON 对象：

- **`t_s`**：时间戳（秒），需与 bag 中位姿可对齐。  
- **`class_name`（或 `class`）**、**`x`/`y`**：目标在 **map 系**下的地面投影点。  
- **`track_id`**：可选。  
- **`confidence`**：可选。

## Rosbag（ROS1，平面 2D）

- **ROS 版本**：仅对 **ROS1** ``.bag``（v2.0）做离线解析（依赖 `rosbags`，无需本机装 ROS）。
- **推荐话题**：**`/odom`**，消息类型 **`nav_msgs/Odometry`**。也支持 `geometry_msgs/PoseStamped` 等含 `Pose` 的类型（见 `rosbag_ros1.extract_pose_xy_yaw`）。
- **2D 语义**：只用 **`x`、`y`**（及内部 **`yaw`**）；与检测区多边形、`detections.jsonl` 中的目标地面点须在 **同一平面坐标系**（通常为 **`map`**）。若 bag 里 `/odom` 在 **`odom`** 帧而分区在 **`map`** 帧，需自行保证录制时已对齐、或换用 **`map` 系** 位姿话题（例如融合定位发布的里程计）。
- ** Inspect**：`py exploration/scripts/inspect_rosbag1.py your.bag`

## 一键栈

```text
py exploration/scripts/run_semantic_stack.py --survey ... --bag ... --topic /odom --detections ... --out ...
```

可选：`--priors-json`（按 `zone_id` 指定**初始权重**）、`--rect-priors-json`（在**均匀格**上对某轴对齐矩形加薪）、`--cell-size`、`--merge-radius`。

输出 **`out_semantic_plan.json`**：`zone_weights`、`rect_weights`、**`waypoints_xy`**（闭合回路中心点序列）。后续可接 **ROS1 move_base** 或 ROS2 Nav2，并结合实际占据栅格做碰撞检查。

## 与 `regions.json` 的衔接

长方形分区文件可由 **`exploration.regions_bridge.load_zones_from_regions_json`** 转成 `DetectionZone`，再**人工或脚本**补上 **`coverage`** 段写入 `survey_zones.json`。

## 初始权重范围

- ``InitialRegionWeight`` 与 ``base_weight_hint`` 在聚合前会钳位到 ``INITIAL_WEIGHT_FLOOR``..``INITIAL_WEIGHT_CEIL``（默认 **0.35 ~ 0.85**），未给时用 ``INITIAL_WEIGHT_DEFAULT``（**0.55**）。  
- 探测完成后的**框权重**仍由 ``AggregationParams.weight_floor`` / ``weight_ceil``（默认 **0.15 ~ 1.0**）约束，以免与导航代价尺度脱节。

## C++ 加速（可选）

大规模均匀格下，**闭合回路**（最近邻 + 2-opt）可编译 ``../exploration_core_cpp`` 中的 ``rbcv_plan_tour``，并设置环境变量 **`RBCV_EXPLORATION_PLANNER_EXE`**。未设置时 **自动使用纯 Python**，结果等价。

## 其它

- 无 rosbag 时：用 `scripts/run_semantic_stack.py` 省略 bag/检测三参数即可自检网格与路径。  
- 目录总览：**`src/WORKSPACE_LAYOUT.md`**
