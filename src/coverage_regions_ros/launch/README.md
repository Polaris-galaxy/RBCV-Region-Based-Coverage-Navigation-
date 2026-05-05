# launch

| 文件 | 说明 |
|------|------|
| `publish_partition_labels.launch` | 启动 `partition_labels_publisher`，默认分区文件 `$(find map_tools)/maps/regions_map7.example.json` |

使用前请先启动 **`map_server`** 或其它发布 `/map` 的节点。

示例：

```bash
roslaunch coverage_regions_ros publish_partition_labels.launch regions_json:=$(rospack find map_tools)/maps/regions_map7.example.json
```

（路径按实际安装位置调整。）
