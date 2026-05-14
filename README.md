
# Washbot ROS2 Workspace

本仓库用于洗车机器人算法与上位机软件开发，当前主要包含：

- 洗车机器人 Gazebo / ROS2 仿真环境
- 机器人 URDF / xacro 描述文件
- 底盘串口驱动节点 `washbot_base_driver`
- 假下位机测试程序 `fake_mcu.py`
- 真实下位机测试脚本
- 雷达 SDK 第三方代码
- 洗车任务决策与巡航节点

当前项目暂时不使用 Docker，推荐在本机 Ubuntu 22.04 + ROS2 Humble 环境下直接运行。

---

## 1. Current Status

当前已经完成：

- ROS2 C++ 底盘串口驱动节点
- `/cmd_vel` 到串口速度帧的转换
- 串口 odom 反馈帧解析
- `/odom` 发布
- `odom -> base_link` TF 发布
- `/base_status` 发布
- `fake_mcu.py` 虚拟下位机测试
- 虚拟串口闭环测试脚本
- 真实下位机串口测试脚本
- 仿真包、描述包、决策包上传
- 雷达 SDK 整理到 `third_party/unilidar_sdk2`

暂时未完成：

- 真实下位机实机联调
- 真实电机速度方向校准
- 真实编码器 / 里程计校准
- 实机雷达数据接入 ROS2
- SLAM / Nav2 完整闭环
- 洗车任务完整状态机

---

## 2. Repository Structure

```text
ros2_ws/
├── src/
│   ├── washbot_base_driver/      # ROS2 C++ 底盘串口驱动
│   ├── washbot_description/      # 机器人 URDF / xacro / launch
│   ├── washbot_decision/         # 洗车任务决策与巡航节点
│   └── carwash_sim/              # 洗车场景仿真 world / mesh
│
├── third_party/
│   └── unilidar_sdk2/            # 雷达 SDK 与 ROS/ROS2 驱动
│
├── tools/
│   └── fake_mcu.py               # 假下位机，用于无硬件测试
│
├── scripts/
│   ├── test_base_driver_fake_mcu.sh
│   └── test_base_driver_real_mcu.sh
│
├── docs/
│   └── base_serial_protocol.md   # 上下位机串口通信协议
│
├── debug.sh                      # 项目调试入口脚本
└── README.md
````

---

## 3. Environment

推荐环境：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo / ros_gz
colcon
Python3
C++17
```

基础依赖安装：

```bash
sudo apt update

sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  socat
```

ROS2 相关依赖建议安装：

```bash
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-teleop-twist-keyboard \
  ros-humble-ros-gz \
  ros-humble-rtabmap-ros
```

---

## 4. To Start

### 4.1 克隆仓库

```bash
cd ~
git clone https://github.com/Aatrox114/washbot_ros2.git ros2_ws
cd ~/ros2_ws
```

如果已经克隆过，直接更新：

```bash
cd ~/ros2_ws
git pull
```

---

### 4.2 初始化 ROS2 环境

每次打开新终端都需要：

```bash
source /opt/ros/humble/setup.bash
```

也可以写入 `~/.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 4.3 安装工作空间依赖

```bash
cd ~/ros2_ws

rosdep update

rosdep install --from-paths src \
  --ignore-src \
  -r -y
```

如果 `rosdep update` 因网络问题失败，可以先跳过，后面根据报错补依赖。

---

### 4.4 编译工作空间

```bash
cd ~/ros2_ws

colcon build --symlink-install
```

编译完成后：

```bash
source ~/ros2_ws/install/setup.bash
```

如果想每次打开终端自动 source：

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Test Base Driver without Real MCU

当前没有真实下位机时，可以使用 `fake_mcu.py` 进行虚拟闭环测试。

该测试链路为：

```text
/cmd_vel
  ↓
base_driver_node
  ↓
虚拟串口 /tmp/washbot_upper
  ↓
fake_mcu.py
  ↓
返回 odom 帧
  ↓
base_driver_node
  ↓
/odom + TF
```

一键测试：

```bash
cd ~/ros2_ws
./scripts/test_base_driver_fake_mcu.sh
```

测试通过时应看到：

```text
base_driver_node 能启动
fake_mcu 能收到 cmd_vel
/odom 能返回
```

可手动检查：

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

---

## 6. Test Base Driver with Real MCU

真实下位机接入后，先查看串口：

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

假设设备为 `/dev/ttyUSB0`，临时给权限：

```bash
sudo chmod 666 /dev/ttyUSB0
```

先运行安全测试，不发运动速度：

```bash
cd ~/ros2_ws
./scripts/test_base_driver_real_mcu.sh /dev/ttyUSB0
```

确认下位机能收到速度帧、CRC 正常、急停可用后，再进行低速运动测试：

```bash
./scripts/test_base_driver_real_mcu.sh /dev/ttyUSB0 --move
```

第一次实机运动测试建议：

```text
轮子离地
速度设得很小
旁边有人看急停
先测试 vx=0 的停车帧
再测试很小的 vx
```

---

## 7. Base Driver

底盘驱动包位置：

```text
src/washbot_base_driver
```

主要功能：

* 订阅 `/cmd_vel`
* 通过串口发送速度控制帧
* 接收下位机 odom 反馈帧
* 发布 `/odom`
* 发布 `odom -> base_link` TF
* 发布 `/base_status`
* `/cmd_vel` 超时后自动发停车指令

启动方式：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch washbot_base_driver base_driver.launch.py
```

指定真实串口：

```bash
ros2 launch washbot_base_driver base_driver.launch.py serial_port:=/dev/ttyUSB0
```

指定虚拟串口：

```bash
ros2 launch washbot_base_driver base_driver.launch.py serial_port:=/tmp/washbot_upper
```

参数文件：

```text
src/washbot_base_driver/config/base_driver.yaml
```

常用参数：

```yaml
serial_port: "/dev/ttyUSB0"
baudrate: 115200
cmd_topic: "/cmd_vel"
odom_topic: "/odom"
status_topic: "/base_status"
odom_frame: "odom"
base_frame: "base_link"
publish_tf: true
cmd_timeout_s: 0.5
send_rate_hz: 50.0
max_vx: 1.0
max_vy: 1.0
max_wz: 2.0
```

---

## 8. Serial Protocol

通信协议文档：

```text
docs/base_serial_protocol.md
```

当前速度控制帧：

```text
AA 55 | 01 | 07 | seq | vx(int16, mm/s) | vy(int16, mm/s) | wz(int16, mrad/s) | CRC16
```

当前里程计反馈帧：

```text
AA 55 | 02 | 14 | seq | x(int32, mm) | y(int32, mm) | yaw(int32, mrad) | vx(int16, mm/s) | vy(int16, mm/s) | wz(int16, mrad/s) | status(uint8) | CRC16
```

基本约定：

```text
字节序：小端
校验：CRC16-MODBUS
默认波特率：115200
上位机发送频率：50Hz
超时保护：0.5 秒无速度帧，下位机应主动停车
```

---

## 9. Lidar SDK

雷达 SDK 放在：

```text
third_party/unilidar_sdk2
```

当前原则：

```text
雷达走网口
底盘控制走串口
二者不要混在同一个驱动节点里
```

雷达 SDK 编译示例：

```bash
cd ~/ros2_ws/third_party/unilidar_sdk2/unitree_lidar_sdk

mkdir -p build
cd build

cmake ..
make -j$(nproc)
```

如果要运行 SDK 示例，先进入 SDK 目录，根据实际雷达连接方式选择 UDP 或串口示例。

---

## 10. Simulation

仿真相关包：

```text
src/carwash_sim
src/washbot_description
src/washbot_decision
```

常用操作：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

如果使用项目调试脚本：

```bash
./debug.sh
```

如果需要单独启动某个 launch 文件，可以先查找：

```bash
find src -name "*.launch.py" | sort
```

然后按实际 launch 文件启动，例如：

```bash
ros2 launch <package_name> <launch_file.launch.py>
```

---





