from __future__ import annotations

import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data


SIMULATION_HZ = 240
TIME_STEP = 1.0 / SIMULATION_HZ
MOVE_DURATION_SECONDS = 3.0
TARGET_POSITION_RAD = 0.5
CONTROLLED_JOINT_NAME = "shoulder_pan_joint"
INITIAL_POSITIONS_RAD = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -math.pi / 2,
    "elbow_joint": math.pi / 2,
    "wrist_1_joint": -math.pi / 2,
    "wrist_2_joint": -math.pi / 2,
    "wrist_3_joint": 0.0,
}


def joint_type_name(joint_type: int) -> str:
    names = {
        p.JOINT_REVOLUTE: "revolute",
        p.JOINT_PRISMATIC: "prismatic",
        p.JOINT_SPHERICAL: "spherical",
        p.JOINT_PLANAR: "planar",
        p.JOINT_FIXED: "fixed",
    }
    return names.get(joint_type, f"unknown({joint_type})")


def main() -> None:
    workspace_root = Path(__file__).resolve().parents[1]
    urdf_path = workspace_root / "models" / "ur5" / "ur5.urdf"
    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"URDF not found: {urdf_path}\n"
            "Run scripts/build_ur5_urdf.py first."
        )

    client_id = p.connect(p.GUI)
    if client_id < 0:
        raise RuntimeError("Could not open the PyBullet GUI.")

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Isolate position control in this first exercise. Gravity and contact
        # are introduced later as separate variables in the grasping scene.
        p.setGravity(0.0, 0.0, 0.0)
        p.setTimeStep(TIME_STEP)
        p.loadURDF("plane.urdf")
        robot_id = p.loadURDF(str(urdf_path), useFixedBase=True)

        p.resetDebugVisualizerCamera(
            cameraDistance=1.8,
            cameraYaw=45.0,
            cameraPitch=-25.0,
            cameraTargetPosition=[0.0, 0.0, 0.35],
        )

        controlled_joint_index = None
        print("\nJoint table")
        print("index | name                         | type       | lower      | upper")
        print("------|------------------------------|------------|------------|-----------")
        for joint_index in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, joint_index)
            name = info[1].decode("utf-8")
            joint_type = joint_type_name(info[2])
            lower, upper = info[8], info[9]
            print(
                f"{joint_index:>5} | {name:<28} | {joint_type:<10} | "
                f"{lower:>10.4f} | {upper:>9.4f}"
            )
            if name == CONTROLLED_JOINT_NAME:
                controlled_joint_index = joint_index

        if controlled_joint_index is None:
            raise RuntimeError(f"Joint not found: {CONTROLLED_JOINT_NAME}")

        movable_joint_indices = [
            index
            for index in range(p.getNumJoints(robot_id))
            if p.getJointInfo(robot_id, index)[2] != p.JOINT_FIXED
        ]
        for joint_index in movable_joint_indices:
            joint_name = p.getJointInfo(robot_id, joint_index)[1].decode("utf-8")
            initial_position = INITIAL_POSITIONS_RAD[joint_name]
            p.resetJointState(robot_id, joint_index, targetValue=initial_position)
            p.setJointMotorControl2(
                robot_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=initial_position,
                force=150.0,
            )

        settle_steps = SIMULATION_HZ
        for _ in range(settle_steps):
            p.stepSimulation()
            time.sleep(TIME_STEP)

        move_steps = round(MOVE_DURATION_SECONDS * SIMULATION_HZ)
        print(
            f"\nMoving {CONTROLLED_JOINT_NAME} from 0.0 to "
            f"{TARGET_POSITION_RAD:.3f} rad "
            f"({math.degrees(TARGET_POSITION_RAD):.2f} deg)..."
        )
        for step in range(move_steps):
            progress = (step + 1) / move_steps
            target = TARGET_POSITION_RAD * progress
            p.setJointMotorControl2(
                robot_id,
                controlled_joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target,
                force=150.0,
                maxVelocity=0.4,
            )
            p.stepSimulation()
            time.sleep(TIME_STEP)

        for _ in range(SIMULATION_HZ):
            p.stepSimulation()
            time.sleep(TIME_STEP)

        actual_position = p.getJointState(robot_id, controlled_joint_index)[0]
        error = TARGET_POSITION_RAD - actual_position
        print(f"Target position: {TARGET_POSITION_RAD:.4f} rad")
        print(f"Actual position: {actual_position:.4f} rad")
        print(f"Position error:  {error:.6f} rad")
        print("Close the PyBullet window to finish.")

        while p.isConnected():
            p.stepSimulation()
            time.sleep(TIME_STEP)
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
