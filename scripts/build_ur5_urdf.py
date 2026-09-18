from __future__ import annotations

import argparse
import os
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro


PACKAGE_NAME = "ur_description"
PACKAGE_URI_PREFIX = f"package://{PACKAGE_NAME}/"


def install_package_lookup(package_root: Path) -> None:
    """Provide the one ament lookup that xacro needs, without installing ROS 2."""
    ament_module = types.ModuleType("ament_index_python")
    packages_module = types.ModuleType("ament_index_python.packages")

    def get_package_share_directory(package_name: str) -> str:
        if package_name != PACKAGE_NAME:
            raise LookupError(f"Unsupported ROS package: {package_name}")
        return str(package_root)

    packages_module.get_package_share_directory = get_package_share_directory
    ament_module.packages = packages_module
    sys.modules["ament_index_python"] = ament_module
    sys.modules["ament_index_python.packages"] = packages_module


def convert_mesh_paths(root: ET.Element, package_root: Path, output_path: Path) -> int:
    relative_package_root = Path(os.path.relpath(package_root, output_path.parent))
    relative_package_uri = relative_package_root.as_posix().rstrip("/") + "/"
    converted = 0

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(PACKAGE_URI_PREFIX):
            mesh.set(
                "filename",
                relative_package_uri + filename.removeprefix(PACKAGE_URI_PREFIX),
            )
            converted += 1

    return converted


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert the ROS-Industrial UR5 xacro into a PyBullet-ready URDF."
    )
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Project root containing ur_description (default: inferred from this script).",
    )
    args = parser.parse_args()

    workspace_root = args.workspace_root.resolve()
    package_root = workspace_root / PACKAGE_NAME
    source_path = package_root / "urdf" / "ur5.xacro"
    output_path = workspace_root / "models" / "ur5" / "ur5.urdf"

    if not source_path.is_file():
        raise FileNotFoundError(f"UR5 xacro not found: {source_path}")

    install_package_lookup(package_root)
    document = xacro.process_file(str(source_path))
    root = ET.fromstring(document.toxml())
    converted_meshes = convert_mesh_paths(root, package_root, output_path)

    root.insert(
        0,
        ET.Comment(
            " Generated from ROS-Industrial universal_robot/ur_description; "
            "retain the upstream BSD 3-Clause license. "
        ),
    )
    ET.indent(root, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)

    joints = list(root.findall("joint"))
    movable_joints = [joint for joint in joints if joint.get("type") != "fixed"]
    print(f"source: {source_path}")
    print(f"output: {output_path}")
    print(f"mesh paths converted: {converted_meshes}")
    print(f"movable joints: {len(movable_joints)}")
    for joint in movable_joints:
        print(f"  {joint.get('name')} ({joint.get('type')})")


if __name__ == "__main__":
    main()
