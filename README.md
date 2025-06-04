# 0. File stucture

zsh_ws/src/
├── diablo_ros2/        # official SDK from github clone
│ │── diablo_ception/         # 本体状态信息：电量，运动状态，IMU，电机
│ │── diablo_common/          # 串口信息处理
│ │── diablo_interaction/     # *控制的基础节点
├── tracking_ws/
│ └── pathtracking/
│   │── scripts/
│   │── resource/
│   │── setup.pyinsta
│   └── package.xml
└── README.md 

# 1. Diablo SDK

> 编译前 修改 diablo_ception/diablo_body/src/diablo_imu.cpp 中 将 frame_id 修正为 base_link

```bash
cd zsh_ws/src
git clone -b basic https://github.com/DDTRobot/diablo_ros2.git
cd ..
colcon build --packages-up-to diablo_ctrl
```

# 2. control DIABLO

Control instructions 控制指令
| 控制ID               | 数值范围   | 备注                 |
| :------------------- | ---------- | -------------------- |
| CMD_GO_FORWARD(0x08) | ±2.0 m/s   | 负数为向后运动       |
| CMD_GO_LEFT(0x04)    | ±2.0 rad/s | 负数为向右运动       |
| CMD_ROLL_RIGHT(0x09) | ±0.2 rad   | 负数为向左运动       |
| CMD_STAND_UP(0x02)   | 0.0        | 机器人站立           |
| CMD_STAND_DOWN(0x12) | 0.0        | 机器人下蹲           |
|                      |            |                      |
| CMD_PITCH_MODE(0x13) | (0.0，1.0) | 位置模式0，速度模式1 |
| CMD_PITCH(0x03)      | ±0.3 pi    | 位置模式0            |
| CMD_PITCH(0x03)      | ±1.8 rad/s | 速度模式1            |
|                      |            |                      |
| CMD_HEIGH_MODE(0x01) | (0.0，1.0) | 位置模式0，速度模式1 |
| CMD_BODY_UP(0x11)    | 0.0~1.0    | 位置模式0            |
| CMD_BODY_UP(0x11)    | ±0.25 m/s  | 速度模式1            |

> 速度模式，机器人将以指令中数值的速度持续运动。
> 位置模式，机器人将以指令中数值代表的固定位置运动。

### 2.1 运行 `diablo_ctrl_node` 获取控制权限

您可以将您的控制指令以自定义msg : `MotionCtrl` 的格式发送到 `/diablo/MotionCmd` 实现控制效果。
同时请先关闭遥控器

```bash
cd zsh_ws
source install/setup.bash
ros2 run diablo_ctrl diablo_ctrl_node

ros2 topic hz /diablo/sensor/Imu # check
```

### 2.2 运行 tracker 节点追踪轨迹
```bash
cd zsh_ws/src
source install/setup.bash
colcon build --packages-select pathtracking

install/pathtracking/bin/wheelodom

# publich odom tf
install/pathtracking/bin/ekfodom # or run in another pc: python -B ekf_odom.py

# start track node
install/pathtracking/bin/diablotrack --ros-args -p control_hz:=100 -p path_dt:=0.1

# publish path to test
install/pathtracking/bin/testtrack --ros-args -p path_length:=0.32 -p point_spacing:=0.02
```