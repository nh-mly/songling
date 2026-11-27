# songlin — Piper 机械臂工作区

> Ubuntu 24.04 + ROS 2 Jazzy ｜ 官方 Python 栈 `pyAgxArm` + `agx_arm_ros`

## 目录

```
songlin/
├── src/
│   ├── agx_arm_ros/        # 官方驱动(4 包,原样不改)
│   ├── piper_tools/        # 安全工具(status/jog/reset/move_to/add_table)
│   └── piper_mujoco/       # MuJoCo 物理仿真(ros2_control 接口)
├── scripts/
│   ├── env.sh              # 一行搞定 ROS+工作区+domain+locale
│   ├── build.sh            # colcon 包装,固定 --base-paths src
│   ├── check.sh            # 模型/包/xacro/mesh 健康检查
│   ├── can_up.sh           # 激活 CAN + 抓帧验证
│   ├── real_arm.sh         # 真机/驱动/MoveIt 一键操作(带门禁)
│   └── grip.sh             # 夹爪开合
├── docs/
│   ├── 命令速查.md
│   └── 踩坑记录.md
├── agx_arm_sim/            # 素材:MuJoCo / Isaac / 5 套 MoveIt 参考(不参与构建)
├── agx_arm_urdf/           # 素材:独立 URDF clone(与子模块重复,可删)
├── pyAgxArm/               # pip 安装源
├── build/  install/  log/  # 产物(不进 git)
└── README.md
```

## 第一次

```bash
sudo apt install -y ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
./scripts/build.sh
source ./scripts/env.sh        # 以后每个终端都用这一行代替手贴五行
./scripts/check.sh
```

`env.sh` 做的事:选对 `setup.zsh`/`setup.bash`、source 工作区、
设 `ROS_DOMAIN_ID=42` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、
`LC_NUMERIC=en_US.UTF-8`、把 `~/.local/bin` 加进 PATH。

**为什么必须隔离网络**:同网段另一台机器跑着 Piper 驱动,会往 `/joint_states`
发 8 关节状态导致 RViz 崩溃,并且它订阅 `/control/joint_states` ——
详见 `docs/踩坑记录.md` 第 1 条。

## 三条运行形态

```bash
# ① 只看模型(不需要硬件、不需要 ros2_control)
ros2 launch agx_arm_description display.launch.py \
  arm_type:=piper effector_type:=agx_gripper \
  control:=true control_topic:=display/joint_states

# ② MoveIt mock(需要 ros2_control + moveit 那 140 个包)
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper

# ④ 真机(需要 USB-CAN + pyAgxArm)
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  arm_type:=piper can_port:=can0 speed_percent:=20
```

> `control_topic` **必须改道**。默认 `control/joint_states` 是真机的命令话题,
> 而 `display.launch.py` 会把滑条值发过去 —— 真机节点同时在跑时,拖滑条 = 驱动真机。

## 真机一键操作

```bash
./scripts/real_arm.sh driver     # 终端A:驱动(常驻,门闸默认关)
./scripts/real_arm.sh enable     # 终端B:使能
./scripts/real_arm.sh home       #        回零(仅当 MoveIt 未跑)
./scripts/real_arm.sh moveit     # 终端C:MoveIt + RViz(常驻,带门闸)
./scripts/real_arm.sh check      # 全链路检查(只读)
./scripts/grip.sh 0.06           # 夹爪张 60mm
```

内置三条安全规则(2026-11-25 事故换来的,详见 `docs/踩坑记录.md` #11-#14):

1. 驱动 `control_enabled:=false` —— 门闸默认关闭
2. `auto_control_gate:=true` —— 门闸节点必须起来
3. 起 MoveIt 前检查真机是否接近零位,不一致就拒绝启动

## MuJoCo 仿真

```bash
ros2 launch piper_mujoco mujoco_moveit.launch.py   # MuJoCo + MoveIt + RViz
ros2 launch piper_mujoco mujoco_sim.launch.py      # 只仿真 + 控制器
```

链路与真机相同：`RViz → move_group → JTC → mujoco_ros2_control → MuJoCo 物理`。
改过的 MJCF 在 `src/piper_mujoco/mjcf/`（`joint7` 改名为 `gripper`，行程对齐 100 mm）。

详细用法（窗口操作、暂停/单步/重置、改模型、常见问题）：[`docs/使用_MuJoCo.md`](docs/使用_MuJoCo.md)

## 工具(piper_tools)

```bash
ros2 run piper_tools status                 # 只读,不动
ros2 run piper_tools jog --joint 6 --delta 5 --dry-run
ros2 run piper_tools jog --joint 6 --delta 5
ros2 run piper_tools reset --yes            # 回零(调 /move_home)
ros2 run piper_tools move_to --xyz 0.25 0.0 0.05 --dry-run   # 末端位姿
ros2 run piper_tools add_table              # 往规划场景加桌面
```

`jog` 存在的理由:`agx_arm_ctrl` 收 `control/joint_states` 是**整组下发**,
缺的关节按 0 处理。手写 `ros2 topic pub` 只发一个关节 = 其余全回零。
`jog` 先读反馈、只改一个、补齐六轴再发。

## 已核实的事实(来自源码,不是文档转述)

| 事实 | 依据 |
|---|---|
| SDK 限位 = URDF 限位,**没有额外余量** | `pyAgxArm/api/constants.py` 与 `piper_description.urdf` 一致 |
| 超限是**静默截断**,不报错 | `piper/default/driver.py` `clamp_joints` |
| `display.launch.py` 默认往 `control/joint_states` 发 | `display.launch.py:158` |
| 夹爪名 `gripper`,单位 m,取绝对值 | `agx_arm_ctrl_single_node.py:37,873` |
| `effort: [0.0]` 会被当成未填,回落 1.0 N | 同上 `:875` |
| `arm_status` 是 30 Hz,不是 200 Hz | 同上 `:358` |

## 末端坐标的两个硬规则

1. **姿态**:工具朝下 = `--rpy 0 180 0`(`180 0 0` 是滚转,不是朝下)
2. **高度**:`--tcp` 默认 0.1425,`--xyz` 是**指尖**;若用 `--tcp 0` 则是**法兰**,
   法兰 z 必须 ≥ 0.16,指尖 z ≥ 0.01

规划场景默认没有桌子,撞桌不会被拦。先 `ros2 run piper_tools add_table`。

## 没做的事(以及为什么)

- **不写 C++ `SystemInterface`**:等于自己实现 CAN 协议。官方已经用
  `JSB → control_topic` 的 remap 把桥做完(`agx_arm_moveit/launch/demo.launch.py:205`)。
- **不铺六包骨架**:我们只有 1 个驱动 + 1 套配置,拆 6 个包会得到 4 个空壳。
- **不装 Gazebo**:官方 `agx_arm_sim` 里的 Gazebo 是 Classic 版,Jazzy 上不可用。
