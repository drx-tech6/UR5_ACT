from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import pybullet as p
import pybullet_data


@dataclass
class Observation:
    qpos: list[float]
    gripper_state: int
    object_position: tuple[float, float, float]
    target_position: tuple[float, float, float]


class MinimalUR5Env:
    JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    HOME_Q = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    CONTROL_HZ = 240
    SUCCESS_DISTANCE = 0.06
    SUCCESS_MAX_LINEAR_SPEED = 0.03
    GRASP_DISTANCE = 0.10

    def __init__(self, gui: bool = True) -> None:
        workspace_root = Path(__file__).resolve().parents[1]
        urdf_path = workspace_root / "models" / "ur5" / "ur5.urdf"
        if not urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        client_mode = p.GUI if gui else p.DIRECT
        self.client_id = p.connect(client_mode)
        if self.client_id < 0:
            raise RuntimeError("Could not connect to PyBullet.")

        self.time_step = 1.0 / self.CONTROL_HZ
        self.robot_id: int | None = None
        self.joint_indices: list[int] = []
        self.tool0_link_index: int | None = None
        self.object_id: int | None = None
        self.target_id: int | None = None
        self.grasp_constraint_id: int | None = None
        self.gripper_state = 0
        self.object_position = (0.55, 0.0, 0.04)
        self.target_position = (0.35, 0.25, 0.04)
        self._urdf_path = urdf_path

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(self.time_step)
        p.setGravity(0.0, 0.0, -9.81)
        if gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.8,
                cameraYaw=45.0,
                cameraPitch=-30.0,
                cameraTargetPosition=[0.3, 0.1, 0.2],
            )

    def reset(self, seed: int | None = None, object_pose: tuple[float, float, float] | None = None) -> Observation:
        del seed  # The fixed scene does not randomize yet.
        if object_pose is not None:
            self.object_position = object_pose

        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(self.time_step)
        p.setGravity(0.0, 0.0, -9.81)
        p.loadURDF("plane.urdf")

        table_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.65, 0.55, 0.025])
        table_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.65, 0.55, 0.025],
            rgbaColor=[0.45, 0.45, 0.45, 1.0],
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=table_shape,
            baseVisualShapeIndex=table_visual,
            # The tabletop occupies z=-0.05..0.0. The robot base and cube
            # therefore rest on its top instead of starting inside it.
            basePosition=[0.3, 0.0, -0.025],
        )

        cube_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.04, 0.04, 0.04])
        cube_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.04, 0.04, 0.04],
            rgbaColor=[0.85, 0.2, 0.1, 1.0],
        )
        self.object_id = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=cube_shape,
            baseVisualShapeIndex=cube_visual,
            basePosition=self.object_position,
        )

        target_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.07, 0.07, 0.002])
        target_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.07, 0.07, 0.002],
            rgbaColor=[0.1, 0.8, 0.2, 0.65],
        )
        self.target_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=target_shape,
            baseVisualShapeIndex=target_visual,
            basePosition=self.target_position,
        )

        self.robot_id = p.loadURDF(str(self._urdf_path), useFixedBase=True)
        self.joint_indices = [
            next(
                index
                for index in range(p.getNumJoints(self.robot_id))
                if p.getJointInfo(self.robot_id, index)[1].decode("utf-8") == name
            )
            for name in self.JOINT_NAMES
        ]
        self.tool0_link_index = next(
            index
            for index in range(p.getNumJoints(self.robot_id))
            if p.getJointInfo(self.robot_id, index)[12].decode("utf-8") == "tool0"
        )
        self.grasp_constraint_id = None
        self.gripper_state = 0
        for joint_index, position in zip(self.joint_indices, self.HOME_Q, strict=True):
            p.resetJointState(self.robot_id, joint_index, position)
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=position,
                force=180.0,
            )

        return self.get_observation()

    def get_tool0_pose(self) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        if self.robot_id is None or self.tool0_link_index is None:
            raise RuntimeError("Call reset() before get_tool0_pose().")
        state = p.getLinkState(
            self.robot_id,
            self.tool0_link_index,
            computeForwardKinematics=True,
        )
        return tuple(state[4]), tuple(state[5])

    def _try_attach_object(self) -> bool:
        if self.robot_id is None or self.object_id is None or self.tool0_link_index is None:
            raise RuntimeError("Call reset() before grasping.")
        if self.grasp_constraint_id is not None:
            return True

        tool_position, tool_orientation = self.get_tool0_pose()
        object_position, object_orientation = p.getBasePositionAndOrientation(self.object_id)
        distance = sum(
            (tool - obj) ** 2
            for tool, obj in zip(tool_position, object_position, strict=True)
        ) ** 0.5
        if distance > self.GRASP_DISTANCE:
            return False

        inverse_tool_position, inverse_tool_orientation = p.invertTransform(
            tool_position,
            tool_orientation,
        )
        relative_position, relative_orientation = p.multiplyTransforms(
            inverse_tool_position,
            inverse_tool_orientation,
            object_position,
            object_orientation,
        )
        self.grasp_constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.tool0_link_index,
            childBodyUniqueId=self.object_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0.0, 0.0, 0.0],
            parentFramePosition=relative_position,
            childFramePosition=[0.0, 0.0, 0.0],
            parentFrameOrientation=relative_orientation,
            childFrameOrientation=[0.0, 0.0, 0.0, 1.0],
        )
        return True

    def _release_object(self) -> None:
        if self.grasp_constraint_id is not None:
            p.removeConstraint(self.grasp_constraint_id)
            self.grasp_constraint_id = None

    def get_observation(self) -> Observation:
        if self.robot_id is None or self.object_id is None:
            raise RuntimeError("Call reset() before get_observation().")
        qpos = [p.getJointState(self.robot_id, index)[0] for index in self.joint_indices]
        object_position = p.getBasePositionAndOrientation(self.object_id)[0]
        return Observation(
            qpos=qpos,
            gripper_state=self.gripper_state,
            object_position=tuple(object_position),
            target_position=self.target_position,
        )

    def step(self, action: list[float], simulation_steps: int = 4) -> Observation:
        if self.robot_id is None:
            raise RuntimeError("Call reset() before step().")
        if len(action) != 7:
            raise ValueError(f"Expected 7 values [q1..q6, gripper], got {len(action)}")

        q_target = action[:6]
        requested_gripper_state = int(action[6] >= 0.5)
        if requested_gripper_state == 0:
            self._release_object()
        self.gripper_state = requested_gripper_state
        p.setJointMotorControlArray(
            self.robot_id,
            self.joint_indices,
            p.POSITION_CONTROL,
            targetPositions=q_target,
            forces=[180.0] * 6,
        )
        for _ in range(simulation_steps):
            p.stepSimulation()
        if self.gripper_state == 1:
            self._try_attach_object()
        return self.get_observation()

    def is_success(self) -> bool:
        if self.object_id is None:
            raise RuntimeError("Call reset() before is_success().")
        object_position = p.getBasePositionAndOrientation(self.object_id)[0]
        object_linear_velocity = p.getBaseVelocity(self.object_id)[0]
        horizontal_distance = ((object_position[0] - self.target_position[0]) ** 2 +
                               (object_position[1] - self.target_position[1]) ** 2) ** 0.5
        linear_speed = sum(value * value for value in object_linear_velocity) ** 0.5
        return (
            horizontal_distance < self.SUCCESS_DISTANCE
            and object_position[2] < 0.12
            and linear_speed < self.SUCCESS_MAX_LINEAR_SPEED
            and self.grasp_constraint_id is None
        )

    def close(self) -> None:
        if p.isConnected(self.client_id):
            p.disconnect(self.client_id)


def main() -> None:
    env = MinimalUR5Env(gui=True)
    try:
        observation = env.reset(seed=0)
        print("initial observation:")
        print(observation)

        simulation_steps_per_action = 4
        action_rate_hz = env.CONTROL_HZ // simulation_steps_per_action
        for _ in range(action_rate_hz):
            observation = env.step(
                env.HOME_Q + [0.0],
                simulation_steps=simulation_steps_per_action,
            )
            time.sleep(simulation_steps_per_action * env.time_step)

        print("after one second at home:")
        print(observation)
        print(f"success at reset: {env.is_success()}")
        print("Close the PyBullet window to finish.")
        while p.isConnected(env.client_id):
            p.stepSimulation()
            time.sleep(env.time_step)
    finally:
        env.close()


if __name__ == "__main__":
    main()
