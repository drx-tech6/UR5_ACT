# 第 1 天：最小 UR5 仿真与设备调查

## 当前进度

- 日期：2026-09-18
- 今日目标：认识 UR5 六个关节，并用位置控制驱动一个关节
- 仿真门槛：无重力单关节实验已通过
- 真实机械臂：未发送运动命令

## 已学习的概念

- 关节空间用 `q = [q1, q2, q3, q4, q5, q6]` 表示，单位为弧度。
- 任务空间描述末端执行器的位置和姿态。
- 正向运动学根据关节角计算末端位姿。
- 逆运动学根据目标末端位姿计算关节角。
- 位置控制发送目标关节角，实际关节角由机器人或仿真器反馈。
- `stepSimulation()` 让仿真时间前进一个时间步。
- 急停用于危险或异常运动，不能作为日常停止程序的方法。

## Windows 环境

| 项目 | 已验证结果 |
|---|---|
| 操作系统 | Windows 11（`10.0.22631`） |
| Conda 环境 | `D:\xuexi\Codex\UR_Robot\.conda\ur5_act` |
| Python | 3.10.21 |
| PyBullet | 3.2.7 |
| Xacro | 2.1.1 |
| PyYAML | 6.0.3 |
| GPU | NVIDIA GeForce RTX 3060 Laptop GPU，6144 MiB |
| 原有 base 环境中的 PyTorch | 2.4.1+cu121，CUDA 可用 |

PyTorch 的 GPU 测试是在原有 `base` 环境中完成的。独立的 `ur5_act`
环境尚未安装和验证 PyTorch，后续训练前再处理。

## UR5 模型记录

| 项目 | 内容 |
|---|---|
| 来源 | ROS-Industrial 的 `universal_robot` 仓库 |
| 下载时所在分支 | `noetic-devel` |
| 下载时页面显示的提交 | `f2f8613` |
| 模型包 | `ur_description` |
| UR5 入口文件 | `urdf/ur5.xacro` |
| 许可证 | BSD 3-Clause |
| 生成的模型 | `models/ur5/ur5.urdf` |

生成的 URDF 中保留了上游许可证说明。发布或分发模型源文件时，必须同时保留
`ur_description/LICENSE` 中的版权声明、许可条件和免责声明。

## PyBullet 关节表

PyBullet 共读取到 10 个关节，其中 6 个是可控制的旋转关节，其余 4 个是固定连接或坐标系。

| PyBullet 索引 | 关节名称 | 类型 | 下限（rad） | 上限（rad） |
|---:|---|---|---:|---:|
| 0 | `base_link-base_link_inertia` | 固定 | 不适用 | 不适用 |
| 1 | `shoulder_pan_joint` | 旋转 | -6.2832 | 6.2832 |
| 2 | `shoulder_lift_joint` | 旋转 | -6.2832 | 6.2832 |
| 3 | `elbow_joint` | 旋转 | -3.1416 | 3.1416 |
| 4 | `wrist_1_joint` | 旋转 | -6.2832 | 6.2832 |
| 5 | `wrist_2_joint` | 旋转 | -6.2832 | 6.2832 |
| 6 | `wrist_3_joint` | 旋转 | -6.2832 | 6.2832 |
| 7 | `wrist_3-flange` | 固定 | 不适用 | 不适用 |
| 8 | `flange-tool0` | 固定 | 不适用 | 不适用 |
| 9 | `base_link-base_fixed_joint` | 固定 | 不适用 | 不适用 |

第一版模型的动作顺序为：

```text
[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, gripper]
```

注意：动作中的第一个值 `q1` 对应 PyBullet 索引 1，而不是索引 0。

## 最小运动实验

- 脚本：`scripts/single_joint_motion.py`
- 控制关节：`shoulder_pan_joint`（PyBullet 索引 1）
- 仿真步长：`1 / 240 s`
- 初始目标：`0.0 rad`
- 最终目标：`0.5 rad`（`28.65 deg`）
- 最终实际位置：`0.5000 rad`
- 最终误差：终端保留六位小数时为 `0.000000 rad`
- 重力：关闭，用于单独验证位置控制链路
- 接触和抓取物理：本实验没有验证

## URSim 调查

| 项目 | 状态 |
|---|---|
| 选择的仿真器型号 | UR5，由用户根据启动快捷方式确认 |
| Universal Robots 软件 | 5.25.2 |
| PolyScope | 74.25.198 |
| Controller | 82.0.35 |
| Firmware | 56.0.73 |
| Baseline | 13.0.406 |
| 主机名 | `localhost` |
| IP 地址 | `0.0.0.0`，网络已禁用 |
| Windows 只读连接 | 未验证 |
| 运动命令 | 未发送 |

URSim 用于了解控制器和 PolyScope。它不模拟 PyBullet 中的桌面、物块、夹爪接触和相机，不能代替抓取物理仿真。

## 真实设备调查

| 设备或条件 | 当前状态 | 还需要确认的内容 |
|---|---|---|
| 机械臂型号 | 铭牌确认是 UR5 | 生产日期 2019-04-01，额定负载 5 kg，最大工作半径 850 mm |
| PolyScope 版本 | 示教器启动画面显示 3.15 | 系统信息中的完整补丁版本尚未查看 |
| 控制器代际 | 旧款 UR5 配合 PolyScope 3.15，基本可判断为 CB3 | 尚未拍摄控制柜铭牌 |
| OmniPicker 完整型号 | 待调查 | 产品铭牌或型号编号 |
| OmniPicker 接口和反馈 | 待调查 | 说明书或经过批准的只读状态验证 |
| Femto Bolt 完整型号 | 待调查 | 产品铭牌和 Windows SDK 版本 |
| 相机安装位置 | 待调查 | 安装说明或照片 |
| 实验室负责人批准 | 待确认 | 明确允许进行的测试范围 |

## 安全检查表

- [x] 第 1 天没有向真实机械臂发送运动命令。
- [x] 已理解急停用于危险情况，不能代替正常停止。
- [x] URSim 网络当前处于禁用状态，没有测试控制连接。
- [x] 已找到控制面板上的实体急停按钮。
- [ ] 在实验室负责人指导下确认急停操作和复位流程。
- [ ] 确认正常停止和人工接管流程。
- [ ] 确认允许的速度、加速度、关节范围和工作空间限制。
- [ ] 检查桌面、周边障碍物和人员隔离区域。
- [ ] 真实运动前验证控制超时和通信中断后的停止行为。
- [ ] 真实机械臂通电运动前获得实验室明确批准。

## 运行命令

```powershell
conda activate "D:\xuexi\Codex\UR_Robot\.conda\ur5_act"
python scripts\build_ur5_urdf.py
python scripts\single_joint_motion.py
```

## 第 1 天验收

- [x] 能说明六关节顺序和弧度单位。
- [x] 能区分关节空间和任务空间。
- [x] 能解释位置控制和仿真步长。
- [x] 已记录模型来源和许可证。
- [x] 已加载 UR5，并打印关节名称、索引和限位。
- [x] 已完成最小单关节位置控制实验。
- [ ] 完成真实设备与实验室安全调查。

第 1 天的仿真任务已经完成。真实设备调查仍在进行，但不影响先开始第 2 天的纯仿真内容。
