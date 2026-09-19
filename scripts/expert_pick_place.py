from __future__ import annotations

import math
import time

import pybullet as p

from minimal_ur5_env import MinimalUR5Env


def solve_ik(env: MinimalUR5Env, position: tuple, orientation: tuple) -> list[float]:
    if env.robot_id is None or env.tool0_link_index is None:
        raise RuntimeError("Environment has not been reset.")
    solution = p.calculateInverseKinematics(
        env.robot_id,
        env.tool0_link_index,
        targetPosition=position,
        targetOrientation=orientation,
        maxNumIterations=300,
        residualThreshold=1e-8,
    )
    q = list(solution[:6])
    if not all(math.isfinite(value) for value in q):
        raise RuntimeError(f"IK returned non-finite values: {q}")
    return q


def execute_joint_segment(
    env: MinimalUR5Env,
    stage: str,
    start_q: list[float],
    goal_q: list[float],
    gripper: float,
    duration_seconds: float,
) -> list[float]:
    steps = max(1, round(duration_seconds * env.CONTROL_HZ))
    for step in range(steps):
        t = (step + 1) / steps
        # Smoothstep keeps velocity zero at both ends of each segment.
        blend = t * t * (3.0 - 2.0 * t)
        q_target = [
            (1.0 - blend) * start + blend * goal
            for start, goal in zip(start_q, goal_q, strict=True)
        ]
        env.step(q_target + [gripper], simulation_steps=1)
        time.sleep(env.time_step)

    actual_q = env.get_observation().qpos
    max_joint_error = max(abs(target - actual) for target, actual in zip(goal_q, actual_q, strict=True))
    tool_position, _ = env.get_tool0_pose()
    print(
        f"{stage:<20} tool0=({tool_position[0]:.3f}, {tool_position[1]:.3f}, "
        f"{tool_position[2]:.3f}) max_joint_error={max_joint_error:.5f}"
    )
    return actual_q


def wait_simulation(env: MinimalUR5Env, seconds: float, q_target: list[float], gripper: float) -> None:
    for _ in range(round(seconds * env.CONTROL_HZ)):
        env.step(q_target + [gripper], simulation_steps=1)
        time.sleep(env.time_step)


def main() -> None:
    env = MinimalUR5Env(gui=True)
    try:
        observation = env.reset(seed=0)
        current_q = observation.qpos
        _, tool_orientation = env.get_tool0_pose()
        object_x, object_y, _ = observation.object_position
        target_x, target_y, _ = observation.target_position

        waypoints = [
            ("move above object", (object_x, object_y, 0.25), 0.0, 3.0),
            ("descend to grasp", (object_x, object_y, 0.12), 0.0, 2.0),
        ]
        for stage, position, gripper, duration in waypoints:
            goal_q = solve_ik(env, position, tool_orientation)
            current_q = execute_joint_segment(
                env, stage, current_q, goal_q, gripper, duration
            )

        print("close gripper       simplified constraint request")
        wait_simulation(env, 0.5, current_q, 1.0)
        if env.grasp_constraint_id is None:
            raise RuntimeError("Simplified grasp failed: tool0 was not close enough to the object.")

        transport_waypoints = [
            ("lift object", (object_x, object_y, 0.28), 1.0, 2.0),
            ("above target", (target_x, target_y, 0.28), 1.0, 3.0),
            # Keep the constrained cube just above the target plate. The tool
            # sits about 0.08 m above the cube center in this simplified grasp.
            ("descend to place", (target_x, target_y, 0.17), 1.0, 2.0),
        ]
        for stage, position, gripper, duration in transport_waypoints:
            goal_q = solve_ik(env, position, tool_orientation)
            current_q = execute_joint_segment(
                env, stage, current_q, goal_q, gripper, duration
            )

        print("open gripper        remove simplified constraint")
        wait_simulation(env, 0.05, current_q, 0.0)
        wait_simulation(env, 1.0, current_q, 0.0)

        home_q = execute_joint_segment(
            env,
            "return home",
            current_q,
            env.HOME_Q,
            0.0,
            3.0,
        )
        wait_simulation(env, 0.5, home_q, 0.0)

        final_observation = env.get_observation()
        object_position = final_observation.object_position
        object_velocity = p.getBaseVelocity(env.object_id)[0]
        object_speed = math.sqrt(sum(value * value for value in object_velocity))
        horizontal_error = math.hypot(
            object_position[0] - target_x,
            object_position[1] - target_y,
        )

        print("\nFinal result")
        print("object position (m): " + ", ".join(f"{value:.4f}" for value in object_position))
        print(f"horizontal target error (m): {horizontal_error:.5f}")
        print(f"object linear speed (m/s): {object_speed:.5f}")
        print(f"simplified constraint active: {env.grasp_constraint_id is not None}")
        print(f"success: {env.is_success()}")
        print("This demo uses simplified constraint grasping, not gripper-contact physics.")
        print("Close the PyBullet window to finish.")

        while p.isConnected(env.client_id):
            p.stepSimulation()
            time.sleep(env.time_step)
    finally:
        env.close()


if __name__ == "__main__":
    main()
