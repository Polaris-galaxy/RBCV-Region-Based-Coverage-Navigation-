# ROS1 桥接（可选）

| 文件 | 作用 |
|------|------|
| `detection_recorder_node.py` | 订阅 **`/rbcv/semantic_detections`**（`std_msgs/String`，单条 JSON），**追加**写入 JSONL，供 `scripts/run_semantic_stack.py --detections` 使用。 |

## 与 RT-DETR 仓库配合

在 **RT-detr** 仓库的 `rbcv_ros/` 中提供：

- **`ros1_rtdetr_rbcv_publisher.py`**：实时相机 → RT-DETR → 发布上述 JSON  
- **`offline_bag_rtdetr_jsonl.py`**：仅 rosbags + PyTorch，从 bag 直接写 JSONL（无需 ROS master）

约定字段与 `exploration/data/detections.example.jsonl` 一致：`t_s`, `class_name`, `x`, `y`, `confidence`, `track_id`。

将本脚本拷入你的 catkin 包 `scripts/` 并 `chmod +x`，或使用 `python detection_recorder_node.py`。
