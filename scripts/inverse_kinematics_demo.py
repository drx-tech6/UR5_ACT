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

TARGET_Z_OFFSET = 0.10


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


def set_joint_positions(robot_id: int, indices: list[int], q: list[float]) -> None:
    for joint_index, position in zip(indices, q, strict=True):
        p.resetJointState(robot_id, joint_index, targetValue=position)


def get_tool0_pose(robot_id: int, tool0_link_index: int) -> tuple[tuple, tuple]:
    state = p.getLinkState(
        robot_id,
        tool0_link_index,
        computeForwardKinematics=True,
    )
    return state[4], state[5]


def quaternion_angle_error(target: tuple, actual: tuple) -> float:
    dot = abs(sum(a * b for a, b in zip(target, actual, strict=True)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


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
        set_joint_positions(robot_id, joint_indices, INITIAL_Q)

        start_position, start_orientation = get_tool0_pose(robot_id, tool0_link_index)
        target_position = (
            start_position[0],
            start_position[1],
            start_position[2] + TARGET_Z_OFFSET,
        )

        solution = p.calculateInverseKinematics(
            robot_id,
            tool0_link_index,
            targetPosition=target_position,
            targetOrientation=start_orientation,
            maxNumIterations=200,
            residualThreshold=1e-8,
        )
        solved_q = list(solution[: len(joint_indices)])

        set_joint_positions(robot_id, joint_indices, solved_q)
        actual_position, actual_orientation = get_tool0_pose(robot_id, tool0_link_index)

        position_error_vector = [
            target - actual
            for target, actual in zip(target_position, actual_position, strict=True)
        ]
        position_error = math.sqrt(
            sum(value * value for value in position_error_vector)
        )
        orientation_error = quaternion_angle_error(
            start_orientation,
            actual_orientation,
        )

        print(f"joint indices: {joint_indices}")
        print(f"tool0 link index: {tool0_link_index}")
        print("\ninitial q (rad):    " + ", ".join(f"{q: .4f}" for q in INITIAL_Q))
        print("solved q (rad):     " + ", ".join(f"{q: .4f}" for q in solved_q))
        print("\nstart position (m):  " + ", ".join(f"{v: .6f}" for v in start_position))
        print("target position (m): " + ", ".join(f"{v: .6f}" for v in target_position))
        print("actual position (m): " + ", ".join(f"{v: .6f}" for v in actual_position))
        print(f"position error (m):  {position_error:.8f}")
        print(f"orientation error (rad): {orientation_error:.8f}")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
