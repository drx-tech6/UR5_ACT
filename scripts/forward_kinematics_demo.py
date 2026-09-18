from __future__ import annotations

import math
from pathlib import Path

import pybullet as p


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

INITIAL_Q = [
    0.0,
    -math.pi / 2,
    math.pi / 2,
    -math.pi / 2,
    -math.pi / 2,
    0.0,
]


def find_indices(robot_id: int) -> tuple[list[int], int]:
    joint_indices: dict[str, int] = {}
    tool0_link_index = -1

    for index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, index)
        joint_name = info[1].decode("utf-8")
        child_link_name = info[12].decode("utf-8")

        if joint_name in JOINT_NAMES:
            joint_indices[joint_name] = index
        if child_link_name == "tool0":
            tool0_link_index = index

    missing = [name for name in JOINT_NAMES if name not in joint_indices]
    if missing:
        raise RuntimeError(f"Missing controllable joints: {missing}")
    if tool0_link_index < 0:
        raise RuntimeError("Could not find the tool0 link.")

    return [joint_indices[name] for name in JOINT_NAMES], tool0_link_index


def set_joint_positions(robot_id: int, joint_indices: list[int], q: list[float]) -> None:
    for joint_index, position in zip(joint_indices, q, strict=True):
        p.resetJointState(robot_id, joint_index, targetValue=position)


def get_tool0_pose(robot_id: int, tool0_link_index: int) -> tuple[tuple, tuple]:
    link_state = p.getLinkState(
        robot_id,
        tool0_link_index,
        computeForwardKinematics=True,
    )
    return link_state[4], link_state[5]


def print_pose(label: str, q: list[float], position: tuple, quaternion: tuple) -> None:
    euler = p.getEulerFromQuaternion(quaternion)
    print(f"\n{label}")
    print("q (rad):      " + ", ".join(f"{value: .4f}" for value in q))
    print("position (m): " + ", ".join(f"{value: .4f}" for value in position))
    print("quaternion:   " + ", ".join(f"{value: .4f}" for value in quaternion))
    print("rpy (rad):    " + ", ".join(f"{value: .4f}" for value in euler))


def main() -> None:
    workspace_root = Path(__file__).resolve().parents[1]
    urdf_path = workspace_root / "models" / "ur5" / "ur5.urdf"
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("Could not connect to PyBullet.")

    try:
        robot_id = p.loadURDF(str(urdf_path), useFixedBase=True)
        joint_indices, tool0_link_index = find_indices(robot_id)

        q_a = INITIAL_Q.copy()
        set_joint_positions(robot_id, joint_indices, q_a)
        position_a, quaternion_a = get_tool0_pose(robot_id, tool0_link_index)

        q_b = INITIAL_Q.copy()
        q_b[0] = 0.5
        set_joint_positions(robot_id, joint_indices, q_b)
        position_b, quaternion_b = get_tool0_pose(robot_id, tool0_link_index)

        print(f"joint indices: {joint_indices}")
        print(f"tool0 link index: {tool0_link_index}")
        print_pose("Pose A: q1 = 0.0 rad", q_a, position_a, quaternion_a)
        print_pose("Pose B: q1 = 0.5 rad", q_b, position_b, quaternion_b)

        delta = [b - a for a, b in zip(position_a, position_b, strict=True)]
        distance = math.sqrt(sum(value * value for value in delta))
        print("\nPosition change B - A (m): " + ", ".join(f"{v: .4f}" for v in delta))
        print(f"Straight-line position change (m): {distance:.4f}")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
