"""解析地图资源目录（开发源码树 vs ``devel/share/map_tools`` vs ``rospack``）."""
from __future__ import annotations

from pathlib import Path


def default_maps_directory() -> Path:
    """返回存放 ``*.yaml`` / ``*.pgm`` 的目录."""
    try:
        import rospkg

        pkg_path = Path(rospkg.RosPack().get_path("map_tools"))
        d = pkg_path / "maps"
        if d.is_dir():
            return d
    except Exception:
        pass

    here = Path(__file__).resolve()
    for anc in here.parents:
        dev_maps = anc / "share" / "map_tools" / "maps"
        if dev_maps.is_dir():
            return dev_maps
        src_maps = anc / "maps"
        if src_maps.is_dir() and list(src_maps.glob("*.yaml")):
            return src_maps

    raise FileNotFoundError(
        "找不到 map_tools 地图目录：请先 catkin_make 并 source devel/setup.bash，"
        "或在开发时将终端 cwd 置于含 src/map_tools/maps 的工作空间树下。"
    )


def default_map_yaml(name: str = "map7") -> Path:
    """示例地图 ``{name}.yaml`` 完整路径."""
    p = default_maps_directory() / f"{name}.yaml"
    if not p.exists():
        raise FileNotFoundError(f"地图 YAML 不存在: {p}")
    return p
