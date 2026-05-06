"""加载真实 ROS 地图并保存预览图.

用法:
    py scripts/preview_real_map.py ../map_tools/maps/map7.yaml
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MAP_TOOLS_MAPS = os.path.abspath(os.path.join(ROOT, "..", "map_tools", "maps"))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from coverage_planner.map_io import load_ros_map  # noqa: E402


def main(yaml_path: str) -> None:
    free, info = load_ros_map(
        yaml_path,
        keep_largest_component=False,
        clean_map_verbose=True,
    )
    h, w = free.shape
    cleaned_free = int(free.sum())
    print(
        f"栅格图像文件 = {info.image_path}\n"
        f"分辨率       = {info.resolution} 米/像素\n"
        f"尺寸         = ({h}, {w}) → 约 {h*info.resolution:.1f} × "
        f"{w*info.resolution:.1f} 米\n"
        f"清洗后可走像素 = {cleaned_free}（占 {cleaned_free/(h*w)*100:.1f}%）\n"
        f"占格阈值     = {info.occupied_thresh}\n"
        f"空闲阈值     = {info.free_thresh}"
    )

    img = (free.astype(np.uint8) * 255)
    out_path = os.path.join(HERE, "地图栅格预览.png")
    try:
        import imageio.v3 as iio

        iio.imwrite(out_path, img)
    except Exception:
        from PIL import Image

        Image.fromarray(img).save(out_path)
    print(f"已保存预览图：{out_path}")


if __name__ == "__main__":
    yaml_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        MAP_TOOLS_MAPS, "map7.yaml"
    )
    main(yaml_path)
