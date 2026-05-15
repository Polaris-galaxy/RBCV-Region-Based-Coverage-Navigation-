# `exploration/` — 语义调查与加权探索导航

与 **`rbcv_disk_coverage/`**（Python 包名 **`coverage_planner`**，几何**可见性圆盘覆盖**）分工如下：

| 层级 | 职责 |
|------|------|
| **几何覆盖** | 障碍栅格 + 长方形分区 → 圆盘观测点（`demo_regions.py` 等） |
| **本包** | 检测框 + **覆盖站位圆**、rosbag **机器人轨迹**、视觉 **JSONL** → 目标计数与去重 → **框权重** → 工作区 **均匀矩形格** → **代价/奖励** → **闭合路径** |

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

## Rosbag

- 话题需能被 `exploration/rosbag_ros1.iter_poses_ros1_topic` 解码（常见 `nav_msgs/Odometry`、`geometry_msgs/PoseStamped`）。  
- ** Inspect**：`py exploration/scripts/inspect_rosbag1.py your.bag`

## 一键栈

```text
py exploration/scripts/run_semantic_stack.py --survey ... --bag ... --topic /odom --detections ... --out ...
```

可选：`--priors-json`（按 `zone_id` 指定**初始权重**）、`--rect-priors-json`（在**均匀格**上对某轴对齐矩形加薪）、`--cell-size`、`--merge-radius`。

输出 **`out_semantic_plan.json`**：`zone_weights`、`rect_weights`、**`waypoints_xy`**（闭合回路中心点序列）。后续可接 Nav2 / 实际占据栅格再做碰撞检查。

## 与 `regions.json` 的衔接

长方形分区文件可由 **`exploration.regions_bridge.load_zones_from_regions_json`** 转成 `DetectionZone`，再**人工或脚本**补上 **`coverage`** 段写入 `survey_zones.json`。

## RT-DETR ↔ RBCV 话题与录制

与你的 **Ultralytics RT-DETR** 仓库并排使用时，请在 **RT-detr/rbcv_ros/** 中部署检测发布；RBCV 侧约定如下。

### 发布（实时，ROS1）

| 话题 | 类型 | 内容 |
|------|------|------|
| `/rbcv/semantic_detections`（可调） | `std_msgs/String` | **每条消息一个 JSON**：`{"t_s","class_name","x","y","confidence","track_id"}`，`x,y` 为 **map** 平面交点（米） |

脚本：`RT-detr/rbcv_ros/ros1_rtdetr_rbcv_publisher.py`（需 TF `map`→光学系 **或** 静态 YAML `~static_T_map_cam_yaml` + 内参）。

### 录制到 JSONL（ROS1）

| 节点 | 作用 |
|------|------|
| `exploration/ros1_bridge/detection_recorder_node.py` | 订阅上述话题，**追加**写入 `*.jsonl`，供 `run_semantic_stack.py --detections` |

### 离线 bag（无 ROS master）

| 脚本 | 作用 |
|------|------|
| `RT-detr/rbcv_ros/offline_bag_rtdetr_jsonl.py` | 仅 **rosbags + PyTorch**，从 bag 图像 topic 直接写 **JSONL** |

外参模板：`RT-detr/rbcv_ros/static_extrinsic.example.yaml`（务必换成真实 **T_map_optical** 与 **K**）。

## 其它

- 无 rosbag 时：用 `scripts/run_semantic_stack.py` 省略 bag/检测三参数即可自检网格与路径。  
- 目录总览：**`src/WORKSPACE_LAYOUT.md`**
