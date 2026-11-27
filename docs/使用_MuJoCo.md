# 使用 MuJoCo 仿真

> 链路与真机完全一致，只把最后一段 `agx_arm_ctrl → CAN` 换成 `MuJoCo 物理`。
>
> ```
> RViz → move_group → FollowJointTrajectory → JTC → mujoco_ros2_control → MuJoCo
>                                                        ↑
>                                          joint_state_broadcaster
> ```

---

## 1. 启动 / 停止

```bash
source ~/107/songlin/scripts/env.sh

# 全套：MuJoCo + 控制器 + MoveIt + RViz
ros2 launch piper_mujoco mujoco_moveit.launch.py

# 只起 MuJoCo + 控制器（不接 MoveIt，省内存）
ros2 launch piper_mujoco mujoco_sim.launch.py

# 无窗口（调试/远程时用）
ros2 launch piper_mujoco mujoco_moveit.launch.py headless:=true
```

**停止**：在启动它的终端按 **Ctrl-C**。

启动约 25 秒就绪，判据：

```bash
ros2 control list_controllers            # arm_controller / gripper_controller / joint_state_broadcaster 全 active
ros2 topic echo /joint_states --once     # 6 轴 + gripper
```

RViz 里 `MotionPlanning` 显示 **Status: Ok** 即可操作。

---

## 2. MuJoCo 窗口怎么操作

| 操作 | 效果 |
|---|---|
| 左键拖动 | 旋转视角 |
| 右键拖动 | 平移视角 |
| 滚轮 | 缩放 |
| 双击物体 | 选中（显示坐标轴） |
| **Ctrl + 右键拖动** | **施加外力**——可以推机械臂、看它倒 |
| `Space`（部分版本） | 暂停 |

仿真是"真实"物理：没使能概念、没急停，推倒了会塌。搞坏了就 `reset_world`（见第 4 节）。

---

## 3. 让它动起来（三种方式）

### A. RViz 规划（推荐，和真机一样）

```
MotionPlanning → 拖交互标记 / Joints 页改值
              → Plan（先看轨迹）
              → Plan & Execute
```

MuJoCo 窗口里会跟着动。

### B. 命令行直接发轨迹

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1,joint2,joint3,joint4,joint5,joint6],
    points: [{positions: [0.0, 0.5, -0.3, 0.0, 0.2, 0.0], time_from_start: {sec: 2}}]}}"
```

### C. 夹爪

```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [gripper],
    points: [{positions: [0.04], time_from_start: {sec: 1}}]}}"
```

> 夹爪关节名是 `gripper`（0~0.10 m）。MJCF 里的 `joint8` 通过 `<equality>` 联动，
> 不需要单独发。

---

## 4. 暂停 / 单步 / 重置

```bash
# 暂停 / 继续
ros2 service call /mujoco_ros2_control_node/set_pause \
  mujoco_ros2_control_msgs/srv/SetPause "{paused: true}"

# 单步 N 步（暂停状态下调试物理用）
ros2 service call /mujoco_ros2_control_node/step_simulation \
  mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 100}"

# 重置到启动时状态
ros2 service call /mujoco_ros2_control_node/reset_world \
  mujoco_ros2_control_msgs/srv/ResetWorld "{keyframe: ''}"
```

`reset_world` 也可以从 MJCF 的 keyframe 恢复（`keyframe: "名字"`），本仓库目前没定义 keyframe。

---

## 5. 看数据

```bash
ros2 topic echo /joint_states --once      # 6 轴 + gripper
ros2 topic hz /clock                      # 仿真时钟
ros2 topic hz /joint_states               # 应为 50 Hz
ros2 control list_hardware_interfaces     # MujocoSystem 的命令/状态接口
ros2 param get /controller_manager update_rate
```

---

## 6. 改模型

| 想改什么 | 改哪个文件 |
|---|---|
| 关节范围、电机增益 `kp/kv` | `src/piper_mujoco/mjcf/piper.xml` |
| 地面、光照、放物体 | `src/piper_mujoco/mjcf/scene.xml` |
| 控制器频率、关节列表 | `src/piper_mujoco/config/mj_controllers.yaml` |
| 关节名映射（MJCF ↔ URDF） | `mjcf/piper.xml` 里的 `<joint name>` 与 `<actuator>` |

改完**重启 launch**（MJCF 只在启动时加载）。

编译：

```bash
~/107/songlin/scripts/build.sh --packages-select piper_mujoco
```

---

## 7. 和真机的差别

| | 仿真 | 真机 |
|---|---|---|
| 硬件接口 | `mujoco_ros2_control/MujocoSystemInterface` | `agx_arm_ctrl`（Python 节点）+ CAN |
| 使能 | 不需要 | 必须 `enable` |
| 急停 | 无，可 `reset_world` | 有按钮 |
| 碰撞后果 | 数字层面，无所谓 | 会撞坏 |
| 时钟 | `use_sim_time=True`，由 `/clock` 驱动 | 系统时间 |

**MoveIt 侧完全一样**：同一份 SRDF、同一套控制器名（`arm_controller` / `gripper_controller`）、同一个 `move_action`。

---

## 8. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 控制器全 `inactive` | 有另一套 ROS 栈在抢 `controller_manager`。先清场（见 `docs/踩坑记录.md` #30） |
| `out of pty devices` | launch 里 `emulate_tty=True`，pty 不够，去掉即可（#29） |
| 规划报自碰撞 `link1-link2` | SRDF 缺 `disable_collisions`，要用 xacro 展开（#26） |
| `No controller_names specified` | `moveit_controllers` 参数要平铺到顶层（#27） |
| RViz 里没有夹爪 | URDF 的 `effector_type` 默认是 `none`（#28） |
| 机械臂站不住/软趴趴 | MuJoCo 里没有"使能"，靠关节 actuator 的 kp 支撑；调 `piper.xml` 的 `kp` |
| 关掉后窗口还在 | 进程没退干净：`pgrep -af mujoco` 看，按 PID kill |

---

## 9. 内部结构

```
src/piper_mujoco/
├── mjcf/                    # 从 agx_arm_sim 复制,已改名
│   ├── piper.xml            # 关节 joint1..6 + gripper(原 joint7) + joint8(linked)
│   ├── scene.xml            # 场景(include piper.xml)
│   └── assets/              # 34 MB obj mesh
├── urdf/
│   └── piper_mj.urdf.xacro  # URDF(给 MoveIt/RViz) + MuJoCo 硬件接口
├── config/
│   ├── mj_controllers.yaml  # JSB + arm_controller(JTC) + gripper_controller(JTC)
│   └── mj_plugins.yaml      # 心跳插件
└── launch/
    ├── mujoco_sim.launch.py    # M1: 仿真 + 控制器
    └── mujoco_moveit.launch.py # M2: 上面 + move_group + RViz
```

**MJCF 改了一处**：`joint7` → `gripper`（含 equality 与 actuator 的引用），
行程 `0.035` → `0.05`，和 URDF 的 100 mm 对齐。

---

## 10. 切换回真机前

MuJoCo 和真机**不能同时跑**：

```bash
pgrep -af 'mujoco|move_group|ros2_control_node'
```

有输出就先 Ctrl-C，确认干净后再起真机：

```bash
~/107/songlin/scripts/real_arm.sh driver
~/107/songlin/scripts/real_arm.sh moveit
```
