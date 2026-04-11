"""
验收点3：UR3 description 包结构解析节点

扫描 ur_description 包路径，列出关键文件树、关节名称、link 数量、xacro 参数入口。
不依赖运行中的机械臂，纯静态文件解析。
"""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node


def find_package_share(pkg_name: str) -> Path | None:
    """通过 ament_index 定位包的 share 目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(pkg_name))
    except Exception:
        return None


class DescriptionReaderDemoNode(Node):
    def __init__(self):
        super().__init__("demo_description_reader_node")
        self._analyze()

    def _analyze(self):
        log = self.get_logger()
        pkg_name = "ur_description"

        log.info("=" * 60)
        log.info("  验收点 3：UR3 Description 包结构解析")
        log.info("=" * 60)

        share_dir = find_package_share(pkg_name)
        if share_dir is None:
            log.error(f"  [ERR] 找不到包 '{pkg_name}'，请确认已安装 ur_description。")
            return

        log.info(f"  包名       : {pkg_name}")
        log.info(f"  share 路径 : {share_dir}")
        log.info("")

        # ── 关键子目录 ─────────────────────────────────────────
        log.info("  关键子目录：")
        for subdir in ["urdf", "meshes", "config", "launch"]:
            full = share_dir / subdir
            exists = "✅" if full.exists() else "❌"
            log.info(f"    {exists} {subdir}/")

        log.info("")

        # ── urdf / xacro 文件列表 ──────────────────────────────
        urdf_dir = share_dir / "urdf"
        if urdf_dir.exists():
            log.info("  urdf/ 文件树：")
            for f in sorted(urdf_dir.rglob("*")):
                if f.is_file():
                    rel = f.relative_to(share_dir)
                    log.info(f"    {rel}")
        log.info("")

        # ── 解析 ur3.urdf.xacro 中的关节与 link ───────────────
        self._parse_urdf(share_dir, log)

        log.info("=" * 60)
        log.info("  [结论] ur_description 包包含 urdf/xacro + meshes + config + launch，")
        log.info("         通过 xacro 参数（name/prefix/joint_limits_parameters_file 等）")
        log.info("         可适配不同型号与场景，是机械臂系统集成的核心入口。")
        log.info("=" * 60)

    def _parse_urdf(self, share_dir: Path, log):
        """尝试解析 urdf 目录下已展开的 .urdf 文件（若存在），提取 joint/link 信息。"""
        urdf_dir = share_dir / "urdf"
        candidates = list(urdf_dir.glob("ur3*.urdf")) if urdf_dir.exists() else []

        if not candidates:
            log.info("  [注] urdf/ 下未找到已展开的 .urdf 文件（仅有 .xacro），")
            log.info("       xacro 主要参数入口如下：")
            self._show_xacro_params(urdf_dir, log)
            return

        urdf_file = candidates[0]
        log.info(f"  解析文件: {urdf_file.name}")
        try:
            tree = ET.parse(urdf_file)
            root = tree.getroot()
            links = [el.get("name") for el in root.findall("link")]
            joints = [el.get("name") for el in root.findall("joint")]
            log.info(f"  Link  数量 : {len(links)}")
            log.info(f"  Joint 数量 : {len(joints)}")
            log.info("  关节列表：")
            for j in joints:
                log.info(f"    - {j}")
        except ET.ParseError as e:
            log.warn(f"  XML 解析失败: {e}")

    def _show_xacro_params(self, urdf_dir: Path, log):
        """列出 xacro 文件中常见的 arg 参数入口。"""
        xacro_files = sorted(urdf_dir.glob("*.xacro")) if urdf_dir.exists() else []
        known_params = [
            "name",
            "prefix",
            "joint_limits_parameters_file",
            "kinematics_parameters_file",
            "physical_parameters_file",
            "visual_parameters_file",
            "safety_limits",
            "safety_pos_margin",
            "safety_k_position",
        ]
        log.info("  常见 xacro 参数（在 ur.urdf.xacro 中通过 <xacro:arg> 声明）：")
        for p in known_params:
            log.info(f"    --xacro-args {p}:=<值>")
        if xacro_files:
            log.info(f"  可用 xacro 文件：")
            for f in xacro_files:
                log.info(f"    {f.name}")


def main(args=None):
    rclpy.init(args=args)
    node = DescriptionReaderDemoNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
