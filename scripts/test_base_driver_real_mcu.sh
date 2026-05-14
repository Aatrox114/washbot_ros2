#!/usr/bin/env bash
set -e

WS="$HOME/ros2_ws"

PORT="${1:-/dev/ttyUSB0}"
MOVE_FLAG="${2:-}"

DRIVER_PID=""

cleanup() {
    echo ""
    echo "[CLEANUP] 正在关闭真实下位机测试..."

    if [ -n "$DRIVER_PID" ]; then
        kill "$DRIVER_PID" 2>/dev/null || true
    fi

    echo "[CLEANUP] 完成"
}

trap cleanup EXIT INT TERM

echo "======================================"
echo "真实下位机串口测试"
echo "串口设备: $PORT"
echo "运动模式: ${MOVE_FLAG:-默认不运动}"
echo "======================================"

echo "[1/8] 检查串口设备是否存在..."

if [ ! -e "$PORT" ]; then
    echo "[ERROR] 找不到串口设备: $PORT"
    echo ""
    echo "你可以先执行："
    echo "  ls /dev/ttyUSB*"
    echo "  ls /dev/ttyACM*"
    echo ""
    echo "然后例如这样运行："
    echo "  ./scripts/test_base_driver_real_mcu.sh /dev/ttyUSB0"
    exit 1
fi

echo "[OK] 串口设备存在: $PORT"

echo "[2/8] 检查串口权限..."

if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo "[WARN] 当前用户可能没有串口读写权限"
    echo "可以临时执行："
    echo "  sudo chmod 666 $PORT"
    echo ""
    echo "长期做法："
    echo "  sudo usermod -aG dialout \$USER"
    echo "然后注销重新登录"
    exit 1
fi

echo "[OK] 串口权限正常"

echo "[3/8] 加载 ROS2 环境..."

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[4/8] 启动 base_driver_node..."

ros2 launch washbot_base_driver base_driver.launch.py serial_port:="$PORT" &
DRIVER_PID=$!

sleep 3

echo "[5/8] 发送停车指令，确认上电后不会乱动..."

timeout 2s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
-r 10 || true

echo "[6/8] 检查 /base_status 是否有输出..."

if timeout 3s ros2 topic echo /base_status --once; then
    echo "[OK] /base_status 有输出"
else
    echo "[WARN] /base_status 暂时没有输出"
    echo "如果下位机还没返回 status，这是正常的"
fi

echo "[7/8] 检查 /odom 是否有输出..."

if timeout 5s ros2 topic echo /odom --once; then
    echo "[OK] /odom 有输出，说明真实下位机已经返回里程计帧"
else
    echo "[WARN] /odom 没有输出"
    echo ""
    echo "可能原因："
    echo "1. 下位机还没有实现 odom 返回帧"
    echo "2. 下位机返回协议和上位机不一致"
    echo "3. CRC16 不一致"
    echo "4. 串口波特率不一致"
    echo "5. 串口线接反或设备选错"
fi

echo "[8/8] 是否进行小速度运动测试..."

if [ "$MOVE_FLAG" = "--move" ]; then
    echo "[MOVE] 发送非常小的前进速度 0.03 m/s，持续 1 秒"
    echo "[MOVE] 第一次实机测试建议轮子离地或底盘架空"

    timeout 1s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    -r 10 || true

    echo "[MOVE] 发送停车指令 2 秒"

    timeout 2s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    -r 10 || true

    echo "[MOVE] 小速度运动测试结束"
else
    echo "[SAFE] 默认不发运动速度"
    echo ""
    echo "确认下位机协议、急停、轮子架空都没问题后，再运行："
    echo "  ./scripts/test_base_driver_real_mcu.sh $PORT --move"
fi

echo ""
echo "======================================"
echo "真实下位机测试流程结束"
echo "======================================"