# piper_tools

三个只做一件事的小工具。设计目标是**在真机上不出事**,不是功能多。

| 命令 | 作用 | 会不会动机械臂 |
|---|---|---|
| `status` | 读关节角 / 夹爪 / arm_status | ❌ 只读 |
| `jog` | 单个关节 ±Δ | ✅ 但默认 dry-run、限幅 15° |
| `reset` | 调官方 `/move_home` 回零 | ✅ 需显式 `--yes` |

## 为什么 jog 不是一行 `ros2 topic pub`

`agx_arm_ctrl` 收 `control/joint_states` 是**整组下发**:

```python
joints = [arm_joints.get(name, 0) for name in self.arm_joint_names]
```

只发一个关节 → 其余五个被当成 **0** → 手臂甩回零位。
`jog` 先读 `feedback/joint_states`,只改目标关节,补齐 6 个再发。

## 用法

```bash
# 先看状态(不需要使能)
ros2 run piper_tools status
ros2 run piper_tools status --watch 1      # 每秒刷新
ros2 run piper_tools status --json

# 点动:务必先 dry-run
ros2 run piper_tools jog --joint 6 --delta 5 --dry-run
ros2 run piper_tools jog --joint 6 --delta 5

# 回零
ros2 run piper_tools reset --dry-run
ros2 run piper_tools reset --yes
```

## 保护措施

- `jog` 单次 `|delta| > 15°` 拒绝执行(要越过加 `--force`)
- `jog` 目标超出 URDF 限位(joint1 ±150° / joint2 0~180° / joint3 -170~0° /
  joint4 ±100° / joint5 ±70° / joint6 ±170°)拒绝执行
- 收不到 6 个关节的反馈就超时退出,不会"盲发"
- `reset` 不带 `--yes` 等同于 dry-run

## 依赖

只要 `rclpy` / `sensor_msgs` / `std_srvs` / `diagnostic_msgs`,**不需要 pyAgxArm**,
所以没接硬件、`pip` 没装也能编译和 `--dry-run`。

## 已知边界

- `reset` 依赖节点已使能;没使能时服务会返回警告,手臂不动
- `jog` 不做轨迹平滑,只发位置目标(和 MoveIt 走的是同一个话题,不要同时用)
- 夹爪控制没做;要夹爪用 `control/joint_states` 里的 `gripper` 字段
