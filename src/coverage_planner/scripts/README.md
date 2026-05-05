# scripts（命令行工具）

按 **操作顺序** 列出。从 `src/coverage_planner` 目录执行。

| 顺序 | 脚本 | 用途 |
|------|------|------|
| 0 | `preview_real_map.py` | 预览 ROS yaml/pgm，确认地图边界（米） |
| 1 | `draw_regions.py` | **交互式划分**：鼠标拖框生成长方形分区 → 保存 JSON |
| 2 | `preview_regions.py` | 把分区 JSON 叠到地图上预览，自检对齐 |
| 3 | `demo_regions.py` | **逐分区圆盘覆盖**：方正→六边形，窄通道→中轴+反射顶点 |

参考路径写法（地图与分区 JSON 都在 `map_tools/maps/`）：

```bash
cd src/coverage_planner

# 0) 预览地图
py scripts/preview_real_map.py ../map_tools/maps/map7.yaml

# 1) 交互划分（保存到 map_tools/maps/my_regions.json）
py scripts/draw_regions.py ../map_tools/maps/map7.yaml ../map_tools/maps/my_regions.json

# 2) 校验对齐
py scripts/preview_regions.py ../map_tools/maps/map7.yaml ../map_tools/maps/my_regions.json

# 3) r=2.5 m 逐区域覆盖
py scripts/demo_regions.py ../map_tools/maps/map7.yaml 2.5 ../map_tools/maps/my_regions.json
```

完整说明见 [`USAGE.md`](../../USAGE.md)（仓库内为 `src/USAGE.md`）。
