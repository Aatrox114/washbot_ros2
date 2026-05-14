#!/usr/bin/env bash

if [ -z "${BASH_VERSION:-}" ]; then
    exec /usr/bin/env bash "$0" "$@"
fi

# =============================================================================
# WashBot Debug Script
# 洗车机器人 Gazebo / 3D SLAM / 2D Map / Nav2 调试工具
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
WS_DIR="${HOME}/ros2_ws"

SIM_PKG="washbot_description"
SIM_LAUNCH="washbot_scan_test.launch.py"
MAP_WORLD="${WS_DIR}/src/carwash_sim/worlds/carwash_map.sdf"
TASK_WORLD="${WS_DIR}/src/carwash_sim/worlds/carwash.sdf"

MAP_DIR="${WS_DIR}/maps"
RTABMAP_DIR="${MAP_DIR}/rtabmap"

DEFAULT_2D_MAP_PREFIX="carwash_rtab_2d"

LAST_2D_MAP_FILE="${MAP_DIR}/.last_2d_map"
LAST_3D_DB_FILE="${RTABMAP_DIR}/.last_3d_db"


# =============================================================================
# 基础函数
# =============================================================================

source_ros2() {
    source /opt/ros/humble/setup.bash

    if [ -f "${WS_DIR}/install/setup.bash" ]; then
        source "${WS_DIR}/install/setup.bash"
    else
        echo -e "${RED}[错误] 未找到 ${WS_DIR}/install/setup.bash${NC}"
        echo -e "${YELLOW}请先执行:${NC}"
        echo "  cd ${WS_DIR}"
        echo "  colcon build --symlink-install"
        echo "  source install/setup.bash"
        exit 1
    fi
}

print_separator() {
    echo -e "${BLUE}=================================================${NC}"
}

print_header() {
    clear
    print_separator
    echo -e "${BOLD}${CYAN}  WashBot 洗车机器人调试工具${NC}"
    echo -e "${CYAN}  Gazebo + 3D GPU Lidar + RTAB-Map + Nav2${NC}"
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

    local tmp_script="/tmp/washbot_debug_${RANDOM}_$$.sh"

    cat > "${tmp_script}" <<EOF
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source '${WS_DIR}/install/setup.bash'
cd '${WS_DIR}'

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
EOF

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

get_latest_2d_map_yaml() {
    if [ -f "${LAST_2D_MAP_FILE}" ]; then
        local last_map
        last_map="$(cat "${LAST_2D_MAP_FILE}")"
        if [ -f "${last_map}" ]; then
            echo "${last_map}"
            return 0
        fi
    fi

    ls -t "${MAP_DIR}"/*.yaml 2>/dev/null | head -1 || true
}

get_latest_3d_db() {
    if [ -f "${LAST_3D_DB_FILE}" ]; then
        local last_db
        last_db="$(cat "${LAST_3D_DB_FILE}")"
        if [ -f "${last_db}" ]; then
            echo "${last_db}"
            return 0
        fi
    fi

    ls -t "${RTABMAP_DIR}"/*.db 2>/dev/null | head -1 || true
}

clean_processes() {
    echo -e "${YELLOW}[清理] 关闭旧的 Gazebo / bridge / slam / rtabmap / rviz / teleop 进程...${NC}"

    pkill -f ros_gz_bridge 2>/dev/null || true
    pkill -f slam_toolbox 2>/dev/null || true
    pkill -f rtabmap 2>/dev/null || true
    pkill -f rgbd_odometry 2>/dev/null || true
    pkill -f robot_state_publisher 2>/dev/null || true
    pkill -f teleop_twist_keyboard 2>/dev/null || true
    pkill -f rviz2 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
    pkill -f "ros_gz_sim" 2>/dev/null || true
    pkill -f nav2 2>/dev/null || true

    sleep 1
    echo -e "${GREEN}[完成] 旧进程清理结束${NC}"
}

# =============================================================================
# RTAB-Map 3D 建图命令生成
# =============================================================================

make_rtabmap_mapping_cmd() {
    local db_path="$1"

    cat <<EOF
mkdir -p '${RTABMAP_DIR}'

ros2 run rtabmap_slam rtabmap \\
  --ros-args \\
  -p use_sim_time:=true \\
  -p frame_id:=base_footprint \\
  -p map_frame_id:=map \\
  -p odom_frame_id:=odom \\
  -p odom_topic:=/odom \\
  -p subscribe_rgb:=false \\
  -p subscribe_depth:=false \\
  -p subscribe_rgbd:=false \\
  -p subscribe_stereo:=false \\
  -p subscribe_scan:=false \\
  -p subscribe_scan_cloud:=true \\
  -p subscribe_odom_info:=false \\
  -p approx_sync:=true \\
  -p sync_queue_size:=50 \\
  -p map_always_update:=true \\
  -p database_path:='${db_path}' \\
  -p Reg/Strategy:="'1'" \\
  -p Grid/Sensor:="'0'" \\
  -p Grid/RangeMax:="'18.0'" \\
  -p Grid/CellSize:="'0.05'" \\
  -p Grid/NormalsSegmentation:="'true'" \\
  -p Grid/NormalK:="'20'" \\
  -p Grid/MaxGroundAngle:="'0.52'" \\
  -p Grid/MaxGroundHeight:="'0.10'" \\
  -p Grid/MaxObstacleHeight:="'2.0'" \\
  -p Grid/ClusterRadius:="'0.10'" \\
  -p Grid/MinClusterSize:="'20'" \\
  -p Icp/MaxCorrespondenceDistance:="'0.3'" \\
  -p Icp/VoxelSize:="'0.05'" \\
  -p Icp/Iterations:="'30'" \\
  --remap scan_cloud:=/scan/points \\
  --remap odom:=/odom
EOF
}

make_rtabmap_localization_cmd() {
    local db_path="$1"

    cat <<EOF
ros2 run rtabmap_slam rtabmap \\
  --ros-args \\
  -p use_sim_time:=true \\
  -p frame_id:=base_footprint \\
  -p map_frame_id:=map \\
  -p odom_frame_id:=odom \\
  -p odom_topic:=/odom \\
  -p subscribe_rgb:=false \\
  -p subscribe_depth:=false \\
  -p subscribe_rgbd:=false \\
  -p subscribe_stereo:=false \\
  -p subscribe_scan:=false \\
  -p subscribe_scan_cloud:=true \\
  -p subscribe_odom_info:=false \\
  -p approx_sync:=true \\
  -p sync_queue_size:=50 \\
  -p map_always_update:=true \\
  -p database_path:='${db_path}' \\
  -p Mem/IncrementalMemory:="'false'" \\
  -p Mem/InitWMWithAllNodes:="'true'" \\
  -p Reg/Strategy:="'1'" \\
  -p Grid/Sensor:="'0'" \\
  -p Grid/RangeMax:="'18.0'" \\
  -p Grid/CellSize:="'0.05'" \\
  -p Grid/NormalsSegmentation:="'true'" \\
  -p Grid/NormalK:="'20'" \\
  -p Grid/MaxGroundAngle:="'0.52'" \\
  -p Grid/MaxGroundHeight:="'0.10'" \\
  -p Grid/MaxObstacleHeight:="'2.0'" \\
  -p Grid/ClusterRadius:="'0.10'" \\
  -p Grid/MinClusterSize:="'20'" \\
  -p Icp/MaxCorrespondenceDistance:="'0.3'" \\
  -p Icp/VoxelSize:="'0.05'" \\
  -p Icp/Iterations:="'30'" \\
  --remap scan_cloud:=/scan/points \\
  --remap odom:=/odom
EOF
}

# =============================================================================
# 1) 一键启动仿真
# =============================================================================

debug_sim_custom() {
    print_header
    echo -e "${GREEN}[模式 1] 一键启动仿真${NC}"
    echo ""

    source_ros2

    if ask_yes_no "启动前是否清理旧进程?" "y"; then
        clean_processes
    fi

    echo ""
    if ask_yes_no "是否启动键盘控制 teleop?" "y"; then
        START_TELEOP="true"
    else
        START_TELEOP="false"
    fi

    if ask_yes_no "是否启动 RViz?" "y"; then
        START_RVIZ="true"
    else
        START_RVIZ="false"
    fi

    echo ""
    echo -e "${CYAN}启动仿真:${NC}"
    echo "  ros2 launch ${SIM_PKG} ${SIM_LAUNCH} start_slam:=false world_file:=${TASK_WORLD}"
    echo ""

    open_terminal "[WashBot] Gazebo 仿真" \
        "ros2 launch ${SIM_PKG} ${SIM_LAUNCH} start_slam:=false world_file:=${TASK_WORLD}"

    sleep 10

    if [ "${START_TELEOP}" = "true" ]; then
        open_terminal "[WashBot] 键盘控制" \
            "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    fi

    if [ "${START_RVIZ}" = "true" ]; then
        open_terminal "[WashBot] RViz" \
            "rviz2"
    fi

    echo -e "${GREEN}已启动所选模块。${NC}"
}

# =============================================================================
# 2) 无车 world 融合建图：RGB-D + 3D 雷达 + RTAB-Map
# =============================================================================

debug_mapping_3d() {
    print_header
    echo -e "${GREEN}[模式 2] 无车 world 融合建图：RGB-D + 3D雷达 + RTAB-Map${NC}"
    echo ""

    source_ros2
    clean_processes

    mkdir -p "${MAP_DIR}"
    mkdir -p "${RTABMAP_DIR}"

    SESSION_ID="$(make_timestamp)"
    RTAB_DB="${RTABMAP_DIR}/carwash_fusion_no_car_${SESSION_ID}.db"
    echo "${RTAB_DB}" > "${LAST_3D_DB_FILE}"

    echo -e "${CYAN}本次无车融合建图数据库:${NC}"
    echo "  ${RTAB_DB}"
    echo ""
    echo -e "${YELLOW}本模式使用无车 world:${NC}"
    echo "  ${MAP_WORLD}"
    echo ""
    echo -e "${YELLOW}说明:${NC}"
    echo "  1) 本模式不会启动 slam_toolbox"
    echo "  2) 使用 RTAB-Map 融合建图"
    echo "  3) 本模式自动控制机器人前往中心点并原地扫描"
    echo "  4) 建图完成后选择模式 3 保存 2D 映射"
    echo ""

    open_terminal "[WashBot] 无车 Gazebo 仿真" \
        "ros2 launch ${SIM_PKG} ${SIM_LAUNCH} start_slam:=false world_file:=${MAP_WORLD}"

    sleep 12

    open_terminal "[WashBot] RTAB-Map 融合建图" \
        "mkdir -p ${RTABMAP_DIR}
ros2 launch washbot_description washbot_rtabmap_fusion.launch.py \
  database_path:=${RTAB_DB}"

    sleep 4

    open_terminal "[WashBot] 自动中心扫描建图决策" \
    "ros2 run washbot_decision center_scan_odom_node \
  --ros-args \
  -p config_file:=/home/vab/ros2_ws/src/washbot_decision/config/wash_task.yaml"

    sleep 2

    open_terminal "[WashBot] RViz 融合建图查看" \
        "rviz2"

    echo ""
    echo -e "${GREEN}无车融合建图模式已启动。${NC}"
    echo ""
    echo -e "${YELLOW}RViz 设置建议:${NC}"
    echo "  Fixed Frame: map"
    echo "  Add -> TF"
    echo "  Add -> Map，Topic: /map"
    echo "  Add -> PointCloud2，Topic: /cloud_map"
    echo "  Add -> PointCloud2，Topic: /cloud_obstacles"
    echo "  Add -> PointCloud2，Topic: /cloud_ground"
    echo "  Add -> PointCloud2，Topic: /scan/points"
    echo "  Add -> PointCloud2，Topic: /camera/points"
    echo "  Add -> Image，Topic: /camera/rgb/image_raw"
    echo "  Add -> Image，Topic: /camera/depth/image_raw"
    echo ""
    echo -e "${YELLOW}建图建议低速运动:${NC}"
    echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.04}, angular: {z: 0.02}}\" -r 10"
    echo ""
    echo -e "${YELLOW}建图完成后，执行:${NC}"
    echo "  bash ./debug.sh 3"
}



# =============================================================================
# 3) 保存当前 /map 的 2D 映射，自动时间命名
# =============================================================================

debug_save_2d_map() {
    print_header
    echo -e "${GREEN}[模式 3] 保存当前 2D 映射 /map，自动按时间命名${NC}"
    echo ""

    source_ros2

    mkdir -p "${MAP_DIR}"

    SESSION_ID="$(make_timestamp)"

    echo -e "${YELLOW}请输入地图前缀，不要带 .yaml 或 .pgm${NC}"
    read -rp "地图前缀 默认 ${DEFAULT_2D_MAP_PREFIX}: " MAP_PREFIX
    MAP_PREFIX="${MAP_PREFIX:-${DEFAULT_2D_MAP_PREFIX}}"

    MAP_PATH="${MAP_DIR}/${MAP_PREFIX}_${SESSION_ID}"

    echo ""
    echo -e "${CYAN}准备保存 2D 映射:${NC}"
    echo "  ${MAP_PATH}.yaml"
    echo "  ${MAP_PATH}.pgm"
    echo ""

    if ! ros2 topic list | grep -q "^/map$"; then
        echo -e "${RED}[错误] 当前没有 /map 话题。${NC}"
        echo -e "${YELLOW}请先执行模式 2，并让机器人运动一段时间。${NC}"
        return 1
    fi

    echo -e "${YELLOW}检查 /map 发布者:${NC}"
    ros2 topic info /map -v || true
    echo ""

    echo -e "${YELLOW}开始保存地图。若长时间卡住，说明 /map 还没有真正发布数据。${NC}"
    ros2 run nav2_map_server map_saver_cli -f "${MAP_PATH}"

    echo "${MAP_PATH}.yaml" > "${LAST_2D_MAP_FILE}"

    echo ""
    echo -e "${GREEN}2D 映射保存完成:${NC}"
    echo "  ${MAP_PATH}.yaml"
    echo "  ${MAP_PATH}.pgm"

    if [ -f "${LAST_3D_DB_FILE}" ]; then
        echo ""
        echo -e "${CYAN}最近一次 3D RTAB-Map 数据库:${NC}"
        cat "${LAST_3D_DB_FILE}"
    fi
}

# =============================================================================
# 4) 导航模式：使用最近保存的 2D 地图
# =============================================================================

debug_navigation() {
    print_header
    echo -e "${GREEN}[模式 4] 使用已有 2D 地图导航 / 路径规划${NC}"
    echo ""

    source_ros2
    clean_processes

    LATEST_MAP_YAML="$(get_latest_2d_map_yaml)"

    if [ -z "${LATEST_MAP_YAML}" ]; then
        LATEST_MAP_YAML="${MAP_DIR}/${DEFAULT_2D_MAP_PREFIX}.yaml"
    fi

    echo ""
    read -rp "地图 yaml 路径 默认 ${LATEST_MAP_YAML}: " MAP_YAML
    MAP_YAML="${MAP_YAML:-${LATEST_MAP_YAML}}"

    if [ ! -f "${MAP_YAML}" ]; then
        echo -e "${RED}[错误] 地图文件不存在: ${MAP_YAML}${NC}"
        echo -e "${YELLOW}请先建图并保存，或输入正确 map.yaml 路径。${NC}"
        return 1
    fi

    open_terminal "[WashBot] Gazebo 仿真" \
        "ros2 launch ${SIM_PKG} ${SIM_LAUNCH} start_slam:=false world_file:=${TASK_WORLD}"

    sleep 12

    open_terminal "[WashBot] Nav2 导航" \
        "ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=${MAP_YAML}"

    sleep 5

    open_terminal "[WashBot] RViz Nav2" \
        "rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"

    echo ""
    echo -e "${GREEN}导航模式已启动。${NC}"
    echo -e "${YELLOW}RViz 操作:${NC}"
    echo "  1) Fixed Frame 设为 map"
    echo "  2) 用 2D Pose Estimate 给机器人初始位姿"
    echo "  3) 用 Nav2 Goal 发送目标点"
}

# =============================================================================
# 5) 仅启动键盘控制
# =============================================================================

debug_teleop() {
    print_header
    echo -e "${GREEN}[模式 5] 仅启动键盘控制${NC}"
    echo ""

    source_ros2

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
}

# =============================================================================
# 6) 仅启动 RViz
# =============================================================================

debug_rviz() {
    print_header
    echo -e "${GREEN}[模式 6] 仅启动 RViz${NC}"
    echo ""

    source_ros2

    rviz2
}

# =============================================================================
# 7) 话题 / TF / 3D 点云检查
# =============================================================================

debug_check() {
    print_header
    echo -e "${GREEN}[模式 7] 话题 / TF / 3D 点云检查${NC}"
    echo ""

    source_ros2

    echo -e "${CYAN}关键话题:${NC}"
    ros2 topic list | grep -E "scan|points|cloud|clock|cmd_vel|odom|tf|joint_states|map|octomap|grid" || true

    echo ""
    echo -e "${CYAN}/clock 发布者:${NC}"
    ros2 topic info /clock -v || true

    echo ""
    echo -e "${CYAN}/scan 频率，5 秒后自动结束:${NC}"
    timeout 5s ros2 topic hz /scan || true

    echo ""
    echo -e "${CYAN}/scan/points 频率，5 秒后自动结束:${NC}"
    timeout 5s ros2 topic hz /scan/points || true

    echo ""
    echo -e "${CYAN}/odom 频率，5 秒后自动结束:${NC}"
    timeout 5s ros2 topic hz /odom || true

    echo ""
    echo -e "${CYAN}TF: odom -> washbot/base_footprint/lidar_sensor，5 秒后自动结束:${NC}"
    timeout 5s ros2 run tf2_ros tf2_echo odom washbot/base_footprint/lidar_sensor || true

    echo ""
    echo -e "${CYAN}/map 信息:${NC}"
    ros2 topic info /map -v || true

    echo ""
    echo -e "${CYAN}RTAB-Map 相关节点:${NC}"
    ros2 node list | grep -E "rtab|rgbd" || true

    echo ""
    echo -e "${CYAN}生成 TF 树 frames.pdf:${NC}"
    ros2 run tf2_tools view_frames || true

    echo ""
    echo -e "${GREEN}检查完成。${NC}"
}

# =============================================================================
# 8) 清理所有相关进程
# =============================================================================

debug_clean() {
    print_header
    echo -e "${GREEN}[模式 8] 清理进程${NC}"
    echo ""

    clean_processes
}

# =============================================================================
# 9) 快速驾驶：仿真 + 键盘 + RViz，不开建图
# =============================================================================

debug_quick_drive() {
    print_header
    echo -e "${GREEN}[模式 9] 快速驾驶模式：Gazebo + Teleop + RViz${NC}"
    echo ""

    source_ros2
    clean_processes

    open_terminal "[WashBot] Gazebo 仿真" \
        "ros2 launch ${SIM_PKG} ${SIM_LAUNCH} start_slam:=false world_file:=${TASK_WORLD}"

    sleep 10

    open_terminal "[WashBot] 键盘控制" \
        "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

    sleep 2

    open_terminal "[WashBot] RViz" \
        "rviz2"

    echo -e "${GREEN}快速驾驶模式已启动。${NC}"
}

# =============================================================================
# 10) 查看最近保存的 3D DB
# =============================================================================

debug_view_db() {
    print_header
    echo -e "${GREEN}[模式 10] 查看最近保存的 RTAB-Map 3D 数据库${NC}"
    echo ""

    source_ros2

    LATEST_DB="$(get_latest_3d_db)"

    if [ -z "${LATEST_DB}" ]; then
        echo -e "${RED}[错误] 没有找到 .db 文件。${NC}"
        echo -e "${YELLOW}请先执行模式 2 进行 3D 建图。${NC}"
        return 1
    fi

    echo -e "${CYAN}最近的 3D DB:${NC}"
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
# 主菜单
# =============================================================================


# =============================================================================
# 11) 有车 world 融合建图：RGB-D + 3D 雷达 + RTAB-Map
# =============================================================================

debug_mapping_rgbd() {
    print_header
    echo -e "${GREEN}[模式 11] 有车 world 融合建图：RGB-D + 3D雷达 + RTAB-Map${NC}"
    echo ""

    source_ros2
    clean_processes

    mkdir -p "${MAP_DIR}"
    mkdir -p "${RTABMAP_DIR}"

    SESSION_ID="$(make_timestamp)"
    RTAB_DB="${RTABMAP_DIR}/carwash_fusion_with_car_${SESSION_ID}.db"
    echo "${RTAB_DB}" > "${LAST_3D_DB_FILE}"

    echo -e "${CYAN}本次有车融合建图数据库:${NC}"
    echo "  ${RTAB_DB}"
    echo ""
    echo -e "${YELLOW}本模式使用有车 world:${NC}"
    echo "  ${TASK_WORLD}"
    echo ""
    echo -e "${YELLOW}说明:${NC}"
    echo "  1) 本模式用于调试有车场景下的 RGB-D + 3D 雷达融合建图效果"
    echo "  2) 正式导航地图仍建议用模式 2 的无车 world 建图结果"
    echo "  3) 输入传感器：RGB-D 相机 + 3D 雷达 + /odom"
    echo ""

    open_terminal "[WashBot] 有车 Gazebo 仿真" \
        "ros2 launch ${SIM_PKG} ${SIM_LAUNCH} start_slam:=false world_file:=${TASK_WORLD}"

    sleep 12

    open_terminal "[WashBot] RTAB-Map 有车融合建图" \
        "mkdir -p ${RTABMAP_DIR}
ros2 launch washbot_description washbot_rtabmap_fusion.launch.py \
  database_path:=${RTAB_DB}"

    sleep 4

    open_terminal "[WashBot] 建图键盘控制" \
        "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

    sleep 2

    open_terminal "[WashBot] RViz 有车融合建图查看" \
        "rviz2"

    echo ""
    echo -e "${GREEN}有车融合建图模式已启动。${NC}"
    echo ""
    echo -e "${YELLOW}RViz 设置建议:${NC}"
    echo "  Fixed Frame: map"
    echo "  Add -> TF"
    echo "  Add -> Map，Topic: /map"
    echo "  Add -> PointCloud2，Topic: /cloud_map"
    echo "  Add -> PointCloud2，Topic: /cloud_obstacles"
    echo "  Add -> PointCloud2，Topic: /cloud_ground"
    echo "  Add -> PointCloud2，Topic: /scan/points"
    echo "  Add -> PointCloud2，Topic: /camera/points"
    echo "  Add -> Image，Topic: /camera/rgb/image_raw"
    echo "  Add -> Image，Topic: /camera/depth/image_raw"
}


show_menu() {
    print_header
    echo -e "${BOLD}请选择调试模式:${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} 一键启动仿真    - 有车 world，可选键盘/RViz"
    echo -e "  ${GREEN}2)${NC} 无车融合建图    - 无车 world + RGB-D + 3D雷达 + RTAB-Map"
    echo -e "  ${GREEN}3)${NC} 保存2D映射     - 保存 /map 为时间戳 yaml + pgm"
    echo -e "  ${GREEN}4)${NC} 导航模式        - 有车 world + 最近保存的2D地图 + Nav2"
    echo -e "  ${GREEN}5)${NC} 键盘控制        - 仅启动 teleop_twist_keyboard"
    echo -e "  ${GREEN}6)${NC} RViz            - 仅启动 RViz"
    echo -e "  ${GREEN}7)${NC} 话题/TF检查     - 检查 scan/points/camera/odom/tf/map"
    echo -e "  ${GREEN}8)${NC} 清理进程        - 关闭 Gazebo/bridge/rtabmap/rviz/nav2"
    echo -e "  ${GREEN}9)${NC} 快速驾驶        - 有车 world + 键盘 + RViz，不建图"
    echo -e "  ${GREEN}10)${NC} 查看3D DB      - 打开最近保存的 rtabmap 数据库"
    echo -e "  ${GREEN}11)${NC} 有车融合建图   - 有车 world + RGB-D + 3D雷达 + RTAB-Map"
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
            1) debug_sim_custom ;;
            2) debug_mapping_3d ;;
            3) debug_save_2d_map ;;
            4) debug_navigation ;;
            5) debug_teleop ;;
            6) debug_rviz ;;
            7) debug_check ;;
            8) debug_clean ;;
            9) debug_quick_drive ;;
            10) debug_view_db ;;
            11) debug_mapping_rgbd ;;
            0)
                echo -e "${CYAN}退出 WashBot 调试工具。${NC}"
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
#   bash ./debug.sh 2
#   bash ./debug.sh 3
# =============================================================================

if [ -n "$1" ]; then
    source_ros2
    case "$1" in
        1) debug_sim_custom ;;
        2) debug_mapping_3d ;;
        3) debug_save_2d_map ;;
        4) debug_navigation ;;
        5) debug_teleop ;;
        6) debug_rviz ;;
        7) debug_check ;;
        8) debug_clean ;;
        9) debug_quick_drive ;;
        10) debug_view_db ;;
            11) debug_mapping_rgbd ;;
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
