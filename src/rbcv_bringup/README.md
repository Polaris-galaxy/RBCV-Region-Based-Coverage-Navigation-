# `rbcv_bringup`（ROS1）

本包将 **`map_tools`**（静态 `/map`）、**`coverage_regions_ros`**（分区标签）与 **`exploration`**（语义栈离线脚本）用 **launch** 串成两阶段流水线。

## 依赖（catkin）

- `rospy`
- `map_tools`
- `coverage_regions_ros`

Python 脚本依赖 `src/exploration/`（通过 `sys.path` 指向 `src/`，与是否安装为 pip 包无关）。

## ROS1 位姿话题（2D）

- **阶段二** 默认 `--topic` / `RBCV_TOPIC` 为 **`/odom`**，对应 **`nav_msgs/Odometry`**。
- 离线栈只使用平面 **``x``、``y``**（时间对齐与落入检测框覆盖区）；**航向**仅解码存储，当前覆盖判定不依赖 yaw。
- **`survey_zones.json`**、`detections.jsonl` 的坐标应与该话题在同一 **平面坐标系**（一般均为 **`map`**）。若仅用 **``odom``** 帧录制，请先变换到 **``map``** 或改录 **``map``** 系位姿话题。

## Launch 文件

| 文件 | 作用 |
|------|------|
| **`launch/rbcv_map_and_regions.launch`** | 同时启动静态地图发布与分区标签节点。参数：`map_yaml`、`regions_json`、`frame_id`。 |
| **`launch/rbcv_stage1_partitions_to_survey.launch`** | **阶段一**：在阶段一地图+分区基础上，调用 `regions_to_survey_zones.py`，将长方形分区与圆盘覆盖参数写成 **`survey_zones.json`**（检测框 + `coverage`）。参数：`map_yaml`、`regions_json`、`coverage_radius_m`、`survey_out`。 |
| **`launch/rbcv_stage2_semantic_explore.launch`** | **阶段二**：调用 `rbcv_semantic_stack_cli.py`，读取 `survey_zones.json` 与（可选）rosbag + 视觉 JSONL，输出 **`rbcv_semantic_plan.json`****（闭合回路航点等）。参数见文件内 `<arg>`。 |

### 阶段二环境变量（由 launch 注入）

`rbcv_semantic_stack_cli.py` 在 `RBCV_SEMANTIC_FROM_LAUNCH=1` 时读取：

- `RBCV_SURVEY`：`survey_zones.json` 路径  
- `RBCV_OUT`：输出计划 JSON 路径  
- `RBCV_BAG` / `RBCV_TOPIC` / `RBCV_DET`：可选；若设置 `RBCV_BAG`，则三者需能组成完整语义对齐（详见 `exploration/README.md`）  
- `RBCV_CELL`：可选，对应 `--cell-size`（米）

等价命令行也可直接执行 `exploration/scripts/run_semantic_stack.py`。

## 脚本

| 脚本 | 作用 |
|------|------|
| **`scripts/regions_to_survey_zones.py`** | 分区 + 地图 → 写出带覆盖信息的调查区 JSON。 |
| **`scripts/rbcv_semantic_stack_cli.py`** | 转发到语义栈主入口；支持 roslaunch 环境变量模式。 |

## 与仓库文档的对应关系

- 目录总览：`src/WORKSPACE_LAYOUT.md`  
- 语义数据契约：`src/exploration/README.md`  
- 文件索引：`src/FILES.md`
