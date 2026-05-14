#!/usr/bin/env bash
set -e

WS="$HOME/ros2_ws"

UPPER_PORT="/tmp/washbot_upper"
MCU_PORT="/tmp/washbot_mcu"

SOCAT_PID=""
FAKE_MCU_PID=""
DRIVER_PID=""

cleanup() {
    echo ""
    echo "[CLEANUP] 正在关闭测试进程..."

    if [ -n "$DRIVER_PID" ]; then
        kill "$DRIVER_PID" 2>/dev/null || true
    fi

    if [ -n "$FAKE_MCU_PID" ]; then
        kill "$FAKE_MCU_PID" 2>/dev/null || true
    fi

    if [ -n "$SOCAT_PID" ]; then
        kill "$SOCAT_PID" 2>/dev/null || true
    fi

    rm -f "$UPPER_PORT" "$MCU_PORT"

    echo "[CLEANUP] 完成"
}

trap cleanup EXIT INT TERM

echo "[1/7] 清理旧虚拟串口..."
rm -f "$UPPER_PORT" "$MCU_PORT"

echo "[2/7] 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[3/7] 启动虚拟串口..."
socat -d -d pty,raw,echo=0,link="$UPPER_PORT" pty,raw,echo=0,link="$MCU_PORT" &
SOCAT_PID=$!

sleep 1

echo "[4/7] 启动 fake_mcu..."
python3 "$WS/tools/fake_mcu.py" &
FAKE_MCU_PID=$!

sleep 1

echo "[5/7] 启动 base_driver_node..."
ros2 launch washbot_base_driver base_driver.launch.py serial_port:="$UPPER_PORT" &
DRIVER_PID=$!

sleep 3

echo "[6/7] 发送前进速度 3 秒..."
timeout 3s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
-r 10 || true

echo "[6/7] 发送旋转速度 3 秒..."
timeout 3s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
-r 10 || true

echo "[6/7] 发送停车指令 1 秒..."
timeout 1s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
-r 10 || true

echo "[7/7] 检查 /odom 是否有输出..."
ros2 topic echo /odom --once

echo ""
echo "======================================"
echo "测试完成："
echo "1. base_driver_node 能启动"
echo "2. fake_mcu 能收到 cmd_vel"
echo "3. /odom 能返回"
echo ""
echo "你可以另外手动检查 TF："
echo "ros2 run tf2_ros tf2_echo odom base_link"
echo "======================================"
