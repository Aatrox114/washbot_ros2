#!/usr/bin/env bash

if [ -z "${BASH_VERSION:-}" ]; then
    exec /usr/bin/env bash "$0" "$@"
fi

# =============================================================================
# Unitree L2 RTAB-Map Debug Script
# 宇树 Unitree L2 串口雷达 / ICP Odometry / RTAB-Map 建图调试工具
# =============================================================================

set -e

# ================= 颜色 =================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ================= 基本配置 =================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_ROOT="${SCRIPT_DIR}"
LIDAR_ROS2_WS="${LIDAR_ROOT}/unitree_lidar_ros2"
LIDAR_SDK="${LIDAR_ROOT}/unitree_lidar_sdk"

SERIAL_PORT="/dev/ttyACM0"
SERIAL_BAUDRATE="4000000"

MAP_DIR="${LIDAR_ROOT}/maps"
RTABMAP_DIR="${MAP_DIR}/rtabmap"
LOG_DIR="${LIDAR_ROOT}/logs"

LAST_DB_FILE="${RTABMAP_DIR}/.last_rtabmap_db"

# Unitree L2 frame
LIDAR_FRAME="unilidar_lidar"
UNITREE_ROOT_FRAME="unilidar_imu_initial"
UNITREE_IMU_FRAME="unilidar_imu"

# SLAM frame
ODOM_FRAME="odom"
MAP_FRAME="map"

# 是否在一键建图结束时自动启动 RViz
DEFAULT_START_RVIZ="y"

# 是否在清理时停止雷达旋转
DEFAULT_STOP_LIDAR_WHEN_CLEAN="n"


# =============================================================================
# 基础函数
# =============================================================================

source_ros2() {
    source /opt/ros/humble/setup.bash

    if [ -f "${LIDAR_ROS2_WS}/install/setup.bash" ]; then
        source "${LIDAR_ROS2_WS}/install/setup.bash"
    fi
}

print_separator() {
    echo -e "${BLUE}=================================================${NC}"
}

print_header() {
    clear
    print_separator
    echo -e "${BOLD}${CYAN}  Unitree L2 RTAB-Map 调试工具${NC}"
    echo -e "${CYAN}  Serial Lidar + ICP Odometry + RTAB-Map Mapping${NC}"
    print_separator
    echo ""
}

check_terminal() {
    if ! command -v gnome-terminal >/dev/null 2>&1; then
        echo -e "${RED}[错误] 未找到 gnome-terminal${NC}"
        echo -e "${YELLOW}请安装:${NC}"
        echo "  sudo apt install gnome-terminal"
        exit 1
    fi
}

open_terminal() {
    local title="$1"
    local cmd="$2"

    check_terminal

    local tmp_script="/tmp/unitree_l2_debug_${RANDOM}_$$.sh"

    cat > "${tmp_script}" <<EOF2
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash

if [ -f "${LIDAR_ROS2_WS}/install/setup.bash" ]; then
    source "${LIDAR_ROS2_WS}/install/setup.bash"
fi

cd "${LIDAR_ROOT}"

echo "=== ${title} ==="
echo "启动命令:"
cat <<'CMD_EOF'
${cmd}
CMD_EOF
echo

${cmd}

echo
echo "进程已结束，按 Ctrl+D 关闭终端"
exec bash
EOF2

    chmod +x "${tmp_script}"
    gnome-terminal --title="${title}" -- bash "${tmp_script}" &
}

ask_yes_no() {
    local prompt="$1"
    local default="$2"
    local ans

    if [ "${default}" = "y" ]; then
        read -rp "${prompt} [Y/n]: " ans
        ans="${ans:-y}"
    else
        read -rp "${prompt} [y/N]: " ans
        ans="${ans:-n}"
    fi

    case "${ans}" in
        y|Y|yes|YES|Yes) return 0 ;;
        *) return 1 ;;
    esac
}

make_timestamp() {
    date +"%Y%m%d_%H%M%S"
}

check_serial_device() {
    echo -e "${CYAN}[检查] 当前串口设备:${NC}"
    ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
    echo ""

    if [ ! -e "${SERIAL_PORT}" ]; then
        echo -e "${RED}[错误] 未找到串口设备: ${SERIAL_PORT}${NC}"
        echo -e "${YELLOW}请确认:${NC}"
        echo "  1) 雷达串口线是否插好"
        echo "  2) 设备名是否不是 ${SERIAL_PORT}"
        echo "  3) 可用 dmesg -w 查看插拔日志"
        return 1
    fi

    if [ ! -r "${SERIAL_PORT}" ] || [ ! -w "${SERIAL_PORT}" ]; then
        echo -e "${YELLOW}[提示] 当前用户可能没有串口权限，尝试 chmod:${NC}"
        echo "  sudo chmod 666 ${SERIAL_PORT}"
        sudo chmod 666 "${SERIAL_PORT}"
    fi

    echo -e "${GREEN}[OK] 串口设备可用: ${SERIAL_PORT}${NC}"
}

check_sdk_tools() {
    local control_bin="${LIDAR_SDK}/bin/lidar_control_serial"

    if [ ! -x "${control_bin}" ]; then
        echo -e "${RED}[错误] 找不到串口雷达控制工具:${NC}"
        echo "  ${control_bin}"
        echo ""
        echo -e "${YELLOW}请先编译 SDK:${NC}"
        echo "  cd ${LIDAR_SDK}"
        echo "  rm -rf build"
        echo "  mkdir build"
        echo "  cd build"
        echo "  cmake .."
        echo "  make -j2"
        return 1
    fi
}

clean_processes() {
    echo -e "${YELLOW}[清理] 关闭旧的 lidar / icp / rtabmap / tf / rviz 进程...${NC}"

    pkill -f unitree_lidar_ros2_node 2>/dev/null || true
    pkill -f icp_odometry 2>/dev/null || true
    pkill -f rtabmap 2>/dev/null || true
    pkill -f rtabmap_viz 2>/dev/null || true
    pkill -f rviz2 2>/dev/null || true

    pkill -f odom_to_unitree_root_tf 2>/dev/null || true
    pkill -f odom_tf_bridge 2>/dev/null || true
    pkill -f odom_to_tf 2>/dev/null || true
    pkill -f static_transform_publisher 2>/dev/null || true

    pkill -f example_lidar_serial 2>/dev/null || true
    pkill -f lidar_control_serial 2>/dev/null || true

    sleep 1

    echo -e "${GREEN}[完成] 旧进程清理结束${NC}"
}

make_dirs() {
    mkdir -p "${MAP_DIR}"
    mkdir -p "${RTABMAP_DIR}"
    mkdir -p "${LOG_DIR}"
}


# =============================================================================
# 命令生成函数
# =============================================================================

make_lidar_driver_cmd() {
    cat <<EOF2
cd '${LIDAR_ROS2_WS}'
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch unitree_lidar_ros2 launch.py
EOF2
}

make_icp_cmd() {
    cat <<EOF2
source /opt/ros/humble/setup.bash

ros2 run rtabmap_odom icp_odometry \\
  --ros-args \\
  -p frame_id:=${LIDAR_FRAME} \\
  -p odom_frame_id:=${ODOM_FRAME} \\
  -p publish_tf:=false \\
  -p subscribe_scan_cloud:=true \\
  -p subscribe_scan:=false \\
  -p approx_sync:=false \\
  -p queue_size:=30 \\
  -p qos:=2 \\
  -p "Odom/Strategy:='1'" \\
  -p "Odom/GuessMotion:='true'" \\
  -p "Odom/ResetCountdown:='1'" \\
  -p "Icp/PointToPlane:='false'" \\
  -p "Icp/VoxelSize:='0.03'" \\
  -p "Icp/MaxCorrespondenceDistance:='2.0'" \\
  -p "Icp/Iterations:='30'" \\
  -p "Icp/CorrespondenceRatio:='0.05'" \\
  -p "Reg/Strategy:='1'" \\
  -r scan_cloud:=/unilidar/cloud \\
  -r odom:=/odom
EOF2
}

write_tf_bridge_script() {
    cat > /tmp/odom_to_unitree_root_tf.py <<EOF2
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class OdomToUnitreeRootTF(Node):
    def __init__(self):
        super().__init__('odom_to_unitree_root_tf')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        self.br = TransformBroadcaster(self)
        self.count = 0

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )

        self.get_logger().info('started, waiting for /odom...')

    def odom_callback(self, msg):
        self.count += 1

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        # 关键:
        # Unitree 驱动已经发布:
        #   ${UNITREE_ROOT_FRAME} -> ${UNITREE_IMU_FRAME} -> ${LIDAR_FRAME}
        #
        # 所以这里不要发布 odom -> ${LIDAR_FRAME}
        # 而是发布:
        #   ${ODOM_FRAME} -> ${UNITREE_ROOT_FRAME}
        #
        # 最终 TF 树:
        #   ${MAP_FRAME} -> ${ODOM_FRAME} -> ${UNITREE_ROOT_FRAME} -> ${UNITREE_IMU_FRAME} -> ${LIDAR_FRAME}
        t.header.frame_id = '${ODOM_FRAME}'
        t.child_frame_id = '${UNITREE_ROOT_FRAME}'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

        if self.count % 10 == 1:
            self.get_logger().info(
                f'published TF ${ODOM_FRAME} -> ${UNITREE_ROOT_FRAME} | '
                f'x={t.transform.translation.x:.3f}, '
                f'y={t.transform.translation.y:.3f}, '
                f'z={t.transform.translation.z:.3f}'
            )


def main():
    rclpy.init()
    node = OdomToUnitreeRootTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF2

    chmod +x /tmp/odom_to_unitree_root_tf.py
}

make_tf_bridge_cmd() {
    write_tf_bridge_script

    cat <<EOF2
source /opt/ros/humble/setup.bash
python3 /tmp/odom_to_unitree_root_tf.py
EOF2
}

make_rtabmap_mapping_cmd() {
    local db_path="$1"

    cat <<EOF2
source /opt/ros/humble/setup.bash
mkdir -p '${RTABMAP_DIR}'

ros2 run rtabmap_slam rtabmap \\
  --ros-args \\
  -p frame_id:=${LIDAR_FRAME} \\
  -p odom_frame_id:=${ODOM_FRAME} \\
  -p map_frame_id:=${MAP_FRAME} \\
  -p publish_tf:=true \\
  -p wait_for_transform:=1.0 \\
  -p subscribe_rgb:=false \\
  -p subscribe_depth:=false \\
  -p subscribe_scan:=false \\
  -p subscribe_scan_cloud:=true \\
  -p approx_sync:=false \\
  -p queue_size:=30 \\
  -p qos:=2 \\
  -p database_path:='${db_path}' \\
  -p "Mem/IncrementalMemory:='true'" \\
  -p "Mem/InitWMWithAllNodes:='false'" \\
  -p "Rtabmap/DetectionRate:='1.0'" \\
  -p "Reg/Strategy:='1'" \\
  -p "Grid/FromDepth:='false'" \\
  -p "Grid/CellSize:='0.05'" \\
  -p "Grid/RangeMax:='10.0'" \\
  -p "Grid/MaxObstacleHeight:='2.0'" \\
  -p "Grid/MaxGroundHeight:='0.2'" \\
  -r scan_cloud:=/unilidar/cloud \\
  -r odom:=/odom
EOF2
}

make_rviz_cmd() {
    cat <<EOF2
source /opt/ros/humble/setup.bash
rviz2
EOF2
}


# =============================================================================
# 1) 串口雷达控制
# =============================================================================

debug_lidar_control() {
    print_header
    echo -e "${GREEN}[模式 1] 串口雷达控制：start / stop / reset${NC}"
    echo ""

    check_serial_device || return 1
    check_sdk_tools || return 1

    echo -e "${BOLD}请选择雷达控制命令:${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} start  - 启动雷达旋转"
    echo -e "  ${GREEN}2)${NC} stop   - 停止雷达旋转"
    echo -e "  ${GREEN}3)${NC} reset  - 重置雷达"
    echo -e "  ${RED}0)${NC} 返回"
    echo ""

    read -rp "请选择 [0-3]: " choice
    echo ""

    case "${choice}" in
        1) LIDAR_CMD="start" ;;
        2) LIDAR_CMD="stop" ;;
        3) LIDAR_CMD="reset" ;;
        0) return 0 ;;
        *)
            echo -e "${RED}[错误] 无效选择${NC}"
            return 1
            ;;
    esac

    echo -e "${CYAN}执行:${NC}"
    echo "  ${LIDAR_SDK}/bin/lidar_control_serial ${LIDAR_CMD} ${SERIAL_PORT} ${SERIAL_BAUDRATE}"
    echo ""

    "${LIDAR_SDK}/bin/lidar_control_serial" "${LIDAR_CMD}" "${SERIAL_PORT}" "${SERIAL_BAUDRATE}"

    echo ""
    echo -e "${GREEN}[完成] 雷达控制命令执行结束${NC}"
}


# =============================================================================
# 2) 仅启动串口雷达 ROS2 驱动
# =============================================================================

debug_lidar_driver() {
    print_header
    echo -e "${GREEN}[模式 2] 启动 Unitree L2 ROS2 串口雷达驱动${NC}"
    echo ""

    check_serial_device || return 1

    if ask_yes_no "启动前是否清理旧进程?" "y"; then
        clean_processes
    fi

    if ask_yes_no "是否先发送 start 命令启动雷达旋转?" "y"; then
        check_sdk_tools || return 1
        "${LIDAR_SDK}/bin/lidar_control_serial" start "${SERIAL_PORT}" "${SERIAL_BAUDRATE}" || true
    fi

    echo ""
    echo -e "${CYAN}启动雷达 ROS2 驱动:${NC}"
    echo "  ros2 launch unitree_lidar_ros2 launch.py"
    echo ""

    open_terminal "[Unitree L2] ROS2 串口雷达驱动" "$(make_lidar_driver_cmd)"

    echo -e "${GREEN}已启动雷达驱动终端。${NC}"
    echo ""
    echo -e "${YELLOW}检查命令:${NC}"
    echo "  ros2 topic hz /unilidar/cloud"
    echo "  ros2 topic hz /unilidar/imu"
}


# =============================================================================
# 3) 一键启动 RTAB-Map 建图
# =============================================================================

debug_rtabmap_mapping_all() {
    print_header
    echo -e "${GREEN}[模式 3] 一键启动 RTAB-Map 建图${NC}"
    echo ""

    check_serial_device || return 1
    check_sdk_tools || return 1
    make_dirs

    if ask_yes_no "启动前是否清理旧进程?" "y"; then
        clean_processes
    fi

    if ask_yes_no "是否先发送 start 命令启动雷达旋转?" "y"; then
        "${LIDAR_SDK}/bin/lidar_control_serial" start "${SERIAL_PORT}" "${SERIAL_BAUDRATE}" || true
    fi

    if ask_yes_no "是否删除旧的 ~/.ros/rtabmap.db?" "y"; then
        rm -f "${HOME}/.ros/rtabmap.db"
    fi

    SESSION_ID="$(make_timestamp)"
    RTAB_DB="${RTABMAP_DIR}/unitree_l2_rtabmap_${SESSION_ID}.db"
    echo "${RTAB_DB}" > "${LAST_DB_FILE}"

    echo ""
    echo -e "${CYAN}本次 RTAB-Map 数据库:${NC}"
    echo "  ${RTAB_DB}"
    echo ""

    echo -e "${YELLOW}说明:${NC}"
    echo "  1) 使用 /unilidar/cloud 作为 scan_cloud"
    echo "  2) 使用 icp_odometry 由点云生成 /odom"
    echo "  3) TF 桥接发布 ${ODOM_FRAME} -> ${UNITREE_ROOT_FRAME}"
    echo "  4) Unitree 驱动发布 ${UNITREE_ROOT_FRAME} -> ${UNITREE_IMU_FRAME} -> ${LIDAR_FRAME}"
    echo "  5) RTAB-Map 发布 ${MAP_FRAME} -> ${ODOM_FRAME}"
    echo ""

    open_terminal "[Unitree L2] ROS2 雷达驱动" "$(make_lidar_driver_cmd)"

    echo -e "${YELLOW}[等待] 等待雷达话题启动...${NC}"
    sleep 5

    open_terminal "[Unitree L2] ICP Odometry /cloud -> /odom" "$(make_icp_cmd)"

    echo -e "${YELLOW}[等待] 等待 /odom 启动...${NC}"
    sleep 5

    open_terminal "[Unitree L2] TF 桥接 odom -> unilidar_imu_initial" "$(make_tf_bridge_cmd)"

    echo -e "${YELLOW}[等待] 等待 TF 桥接启动...${NC}"
    sleep 3

    open_terminal "[Unitree L2] RTAB-Map 建图" "$(make_rtabmap_mapping_cmd "${RTAB_DB}")"

    echo ""
    if ask_yes_no "是否启动 RViz2 查看地图?" "${DEFAULT_START_RVIZ}"; then
        sleep 2
        open_terminal "[Unitree L2] RViz2 查看地图" "$(make_rviz_cmd)"
    fi

    echo ""
    echo -e "${GREEN}[完成] 建图相关终端已启动。${NC}"
    echo ""
    echo -e "${YELLOW}RViz2 设置建议:${NC}"
    echo "  Fixed Frame: map"
    echo "  Add -> TF"
    echo "  Add -> Odometry    Topic: /odom"
    echo "  Add -> PointCloud2 Topic: /unilidar/cloud"
    echo "  Add -> PointCloud2 Topic: /cloud_map"
    echo "  Add -> Map         Topic: /map"
    echo ""
    echo -e "${YELLOW}手持建图建议:${NC}"
    echo "  先静止 3 秒"
    echo "  慢慢平移 30~50 cm"
    echo "  停一下"
    echo "  慢慢转 10~20 度"
    echo "  不要快速甩动，不要只对着白墙"
}


# =============================================================================
# 4) 仅启动 ICP Odometry
# =============================================================================

debug_icp_only() {
    print_header
    echo -e "${GREEN}[模式 4] 仅启动 ICP Odometry：/unilidar/cloud -> /odom${NC}"
    echo ""

    echo -e "${YELLOW}启动前请确认雷达驱动已经发布 /unilidar/cloud${NC}"
    echo ""

    open_terminal "[Unitree L2] ICP Odometry" "$(make_icp_cmd)"

    echo -e "${GREEN}已启动 ICP Odometry。${NC}"
    echo ""
    echo -e "${YELLOW}检查命令:${NC}"
    echo "  ros2 topic hz /odom"
    echo "  ros2 topic echo /odom --once"
}


# =============================================================================
# 5) 仅启动 TF 桥接
# =============================================================================

debug_tf_bridge_only() {
    print_header
    echo -e "${GREEN}[模式 5] 仅启动 TF 桥接：odom -> unilidar_imu_initial${NC}"
    echo ""

    echo -e "${YELLOW}启动前请确认 /odom 已经发布。${NC}"
    echo ""

    open_terminal "[Unitree L2] TF 桥接" "$(make_tf_bridge_cmd)"

    echo -e "${GREEN}已启动 TF 桥接。${NC}"
    echo ""
    echo -e "${YELLOW}检查命令:${NC}"
    echo "  ros2 run tf2_ros tf2_echo odom unilidar_lidar"
    echo "  ros2 run tf2_tools view_frames"
}


# =============================================================================
# 6) 仅启动 RTAB-Map 建图
# =============================================================================

debug_rtabmap_only() {
    print_header
    echo -e "${GREEN}[模式 6] 仅启动 RTAB-Map 建图节点${NC}"
    echo ""

    make_dirs

    SESSION_ID="$(make_timestamp)"
    RTAB_DB="${RTABMAP_DIR}/unitree_l2_rtabmap_${SESSION_ID}.db"
    echo "${RTAB_DB}" > "${LAST_DB_FILE}"

    echo -e "${CYAN}本次 RTAB-Map 数据库:${NC}"
    echo "  ${RTAB_DB}"
    echo ""

    if ask_yes_no "是否删除旧的 ~/.ros/rtabmap.db?" "n"; then
        rm -f "${HOME}/.ros/rtabmap.db"
    fi

    echo -e "${YELLOW}启动前请确认:${NC}"
    echo "  /unilidar/cloud 有频率"
    echo "  /odom 有频率"
    echo "  tf2_echo odom unilidar_lidar 能通"
    echo ""

    open_terminal "[Unitree L2] RTAB-Map 建图" "$(make_rtabmap_mapping_cmd "${RTAB_DB}")"

    echo -e "${GREEN}已启动 RTAB-Map 建图节点。${NC}"
}


# =============================================================================
# 7) 启动 RViz2
# =============================================================================

debug_rviz() {
    print_header
    echo -e "${GREEN}[模式 7] 启动 RViz2${NC}"
    echo ""

    source_ros2
    rviz2
}


# =============================================================================
# 8) 检查话题 / TF / 地图
# =============================================================================

debug_check() {
    print_header
    echo -e "${GREEN}[模式 8] 检查话题 / TF / 地图${NC}"
    echo ""

    source_ros2

    echo -e "${CYAN}关键话题:${NC}"
    ros2 topic list | grep -E "unilidar|odom|tf|map|cloud|octomap|grid" || true

    echo ""
    echo -e "${CYAN}/unilidar/cloud 频率，5 秒后自动结束:${NC}"
    timeout 5s ros2 topic hz /unilidar/cloud || true

    echo ""
    echo -e "${CYAN}/unilidar/imu 频率，5 秒后自动结束:${NC}"
    timeout 5s ros2 topic hz /unilidar/imu || true

    echo ""
    echo -e "${CYAN}/odom 频率，5 秒后自动结束:${NC}"
    timeout 5s ros2 topic hz /odom || true

    echo ""
    echo -e "${CYAN}TF: odom -> unilidar_lidar，5 秒后自动结束:${NC}"
    timeout 5s ros2 run tf2_ros tf2_echo odom unilidar_lidar || true

    echo ""
    echo -e "${CYAN}TF: map -> odom，5 秒后自动结束:${NC}"
    timeout 5s ros2 run tf2_ros tf2_echo map odom || true

    echo ""
    echo -e "${CYAN}/map 相关话题:${NC}"
    ros2 topic list | grep map || true

    echo ""
    echo -e "${CYAN}/tf 发布者:${NC}"
    ros2 topic info /tf -v || true

    echo ""
    echo -e "${CYAN}/odom 详情:${NC}"
    ros2 topic info /odom -v || true

    echo ""
    echo -e "${CYAN}RTAB-Map 节点:${NC}"
    ros2 node list | grep -E "rtab|icp|odom" || true

    echo ""
    echo -e "${CYAN}生成 TF 树 frames.pdf:${NC}"
    ros2 run tf2_tools view_frames || true

    echo ""
    echo -e "${GREEN}检查完成。${NC}"
}


# =============================================================================
# 9) 保存 RTAB-Map 数据库
# =============================================================================

debug_save_db() {
    print_header
    echo -e "${GREEN}[模式 9] 保存 RTAB-Map 数据库${NC}"
    echo ""

    make_dirs

    if [ -f "${HOME}/.ros/rtabmap.db" ]; then
        DB_SAVE_PATH="${RTABMAP_DIR}/manual_saved_rtabmap_$(make_timestamp).db"
        cp "${HOME}/.ros/rtabmap.db" "${DB_SAVE_PATH}"
        echo "${DB_SAVE_PATH}" > "${LAST_DB_FILE}"

        echo -e "${GREEN}[完成] 已保存:${NC}"
        echo "  ${DB_SAVE_PATH}"
    else
        echo -e "${RED}[错误] 没找到 ${HOME}/.ros/rtabmap.db${NC}"
        echo -e "${YELLOW}如果你启动 RTAB-Map 时指定了 database_path，请查看:${NC}"
        echo "  ${RTABMAP_DIR}"
    fi

    echo ""
    echo -e "${YELLOW}如果 /map 正常，也可以保存 2D 栅格地图:${NC}"
    echo "  ros2 run nav2_map_server map_saver_cli -f ${MAP_DIR}/unitree_l2_map"
}


# =============================================================================
# 10) 查看最近的 RTAB-Map DB
# =============================================================================

debug_view_db() {
    print_header
    echo -e "${GREEN}[模式 10] 查看最近的 RTAB-Map 数据库${NC}"
    echo ""

    make_dirs

    if [ -f "${LAST_DB_FILE}" ]; then
        LATEST_DB="$(cat "${LAST_DB_FILE}")"
    else
        LATEST_DB="$(ls -t "${RTABMAP_DIR}"/*.db 2>/dev/null | head -1 || true)"
    fi

    if [ -z "${LATEST_DB}" ] || [ ! -f "${LATEST_DB}" ]; then
        echo -e "${RED}[错误] 没找到 RTAB-Map .db 文件${NC}"
        return 1
    fi

    echo -e "${CYAN}最近的数据库:${NC}"
    echo "  ${LATEST_DB}"
    echo ""

    if ! command -v rtabmap-databaseViewer >/dev/null 2>&1; then
        echo -e "${RED}[错误] 找不到 rtabmap-databaseViewer${NC}"
        echo -e "${YELLOW}请安装:${NC}"
        echo "  sudo apt install ros-humble-rtabmap-ros"
        return 1
    fi

    rtabmap-databaseViewer "${LATEST_DB}"
}


# =============================================================================
# 11) 清理进程
# =============================================================================

debug_clean() {
    print_header
    echo -e "${GREEN}[模式 11] 清理相关进程${NC}"
    echo ""

    clean_processes

    echo ""
    if ask_yes_no "是否同时停止雷达旋转?" "${DEFAULT_STOP_LIDAR_WHEN_CLEAN}"; then
        check_serial_device || return 1
        check_sdk_tools || return 1
        "${LIDAR_SDK}/bin/lidar_control_serial" stop "${SERIAL_PORT}" "${SERIAL_BAUDRATE}" || true
    fi

    echo -e "${GREEN}[完成] 清理结束${NC}"
}


# =============================================================================
# 主菜单
# =============================================================================

show_menu() {
    print_header
    echo -e "${BOLD}请选择调试模式:${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} 串口雷达控制      - start / stop / reset"
    echo -e "  ${GREEN}2)${NC} 启动雷达驱动      - Unitree L2 ROS2 串口驱动"
    echo -e "  ${GREEN}3)${NC} 一键 RTAB 建图    - 雷达 + ICP + TF桥接 + RTAB-Map + 可选RViz"
    echo -e "  ${GREEN}4)${NC} 仅启动 ICP        - /unilidar/cloud -> /odom"
    echo -e "  ${GREEN}5)${NC} 仅启动 TF桥接     - odom -> unilidar_imu_initial"
    echo -e "  ${GREEN}6)${NC} 仅启动 RTAB-Map   - 使用已有 /cloud + /odom + TF 建图"
    echo -e "  ${GREEN}7)${NC} 启动 RViz2        - 手动查看地图"
    echo -e "  ${GREEN}8)${NC} 话题/TF检查       - 检查 cloud/imu/odom/tf/map"
    echo -e "  ${GREEN}9)${NC} 保存 RTAB DB      - 保存 rtabmap.db"
    echo -e "  ${GREEN}10)${NC} 查看 RTAB DB     - 打开最近数据库"
    echo -e "  ${GREEN}11)${NC} 清理进程         - 关闭 lidar/icp/rtab/tf/rviz"
    echo ""
    echo -e "  ${RED}0)${NC} 退出"
    echo ""
}

main() {
    while true; do
        show_menu
        read -rp "请选择 [0-11]: " choice
        echo ""

        case "${choice}" in
            1) debug_lidar_control ;;
            2) debug_lidar_driver ;;
            3) debug_rtabmap_mapping_all ;;
            4) debug_icp_only ;;
            5) debug_tf_bridge_only ;;
            6) debug_rtabmap_only ;;
            7) debug_rviz ;;
            8) debug_check ;;
            9) debug_save_db ;;
            10) debug_view_db ;;
            11) debug_clean ;;
            0)
                echo -e "${CYAN}退出 Unitree L2 RTAB-Map 调试工具。${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}无效选择，请重试。${NC}"
                ;;
        esac

        echo ""
        read -rp "按 Enter 返回主菜单..."
    done
}


# =============================================================================
# 支持直接传参
# 用法:
#   bash ./debug.sh 3
#   bash ./debug.sh 8
# =============================================================================

if [ -n "$1" ]; then
    case "$1" in
        1) debug_lidar_control ;;
        2) debug_lidar_driver ;;
        3) debug_rtabmap_mapping_all ;;
        4) debug_icp_only ;;
        5) debug_tf_bridge_only ;;
        6) debug_rtabmap_only ;;
        7) debug_rviz ;;
        8) debug_check ;;
        9) debug_save_db ;;
        10) debug_view_db ;;
        11) debug_clean ;;
        0) exit 0 ;;
        *)
            echo -e "${RED}无效参数: $1${NC}"
            echo "用法: $0 [0-11]"
            exit 1
            ;;
    esac
else
    main
fi
