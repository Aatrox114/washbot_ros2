from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    declare_database_path = DeclareLaunchArgument(
        "database_path",
        default_value="/home/yuandong/ros2_ws/maps/rtabmap/carwash_fusion.db",
        description="RTAB-Map database path"
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,

                # ================= 坐标系 =================
                "frame_id": "base_footprint",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "odom_topic": "/odom",

                # 等 TF 的时间放宽一点，避免图像/点云/odom 时间戳略微错位
                "wait_for_transform": 0.5,
                "tf_tolerance": 0.3,
                "tf_delay": 0.05,

                # ================= RGB-D 相机输入 =================
                "subscribe_rgb": True,
                "subscribe_depth": True,
                "subscribe_rgbd": False,
                "subscribe_stereo": False,

                # ================= 3D 雷达点云输入 =================
                "subscribe_scan": False,
                "subscribe_scan_cloud": True,

                # 不订阅 rtabmap 自己的 odom_info
                "subscribe_odom_info": False,

                # 相机和雷达时间戳不完全一致，必须近似同步
                "approx_sync": True,
                "sync_queue_size": 100,
                "topic_queue_size": 50,

                # ================= 数据库 =================
                "database_path": database_path,
                "map_always_update": True,

                # ================= 更新频率 =================
                # 原来 RTAB-Map 可能更新太慢，导致当前白色点云和历史黑色地图不同步
                "Rtabmap/DetectionRate": "5.0",

                # 机器人只要稍微动一点就更新，不要攒很久才更新
                "RGBD/LinearUpdate": "0.02",
                "RGBD/AngularUpdate": "0.02",

                # ================= 融合注册策略 =================
                # 2 = Visual + ICP
                # RGB-D 参与视觉/深度约束，3D 雷达参与 ICP 几何约束
                "Reg/Strategy": "2",

                # 地面机器人，强制 2D 位姿优化
                "Reg/Force3DoF": "true",
                "Optimizer/Slam2D": "true",

                # ================= 栅格地图生成 =================
                # 这里先改成 0：
                # 0 = 主要用 laser / scan_cloud 生成栅格地图
                # RGB-D 仍然参与配准，但不要直接把深度相机地面点也灌进 2D 栅格
                "Grid/Sensor": "0",

                "Grid/RangeMax": "6.0",
                "Grid/CellSize": "0.05",

                # 地面分割
                "Grid/NormalsSegmentation": "true",
                "Grid/NormalK": "20",
                "Grid/MaxGroundAngle": "0.45",
                "Grid/MaxGroundHeight": "0.03",
                "Grid/MaxObstacleHeight": "2.0",

                # 小噪声过滤，数值越大，越不容易把零碎点当障碍
                "Grid/ClusterRadius": "0.15",
                "Grid/MinClusterSize": "150",

                # ================= ICP 参数 =================
                # 原来 0.20 太松，容易把没有对齐的点硬匹配
                "Icp/MaxCorrespondenceDistance": "0.10",

                # 点云降采样，减轻 RTAB-Map 处理压力
                # 原来 0.05 点太密，容易处理不过来
                "Icp/VoxelSize": "0.10",

                "Icp/Iterations": "30",

                # 限制每帧参与处理的点云数量，避免处理延迟导致白黑点云不同步
                "scan_cloud_max_points": 8000,
            }
        ],
        remappings=[
            ("rgb/image", "/camera/rgb/image_raw"),
            ("depth/image", "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/rgb/camera_info"),
            ("scan_cloud", "/scan/points"),
            ("odom", "/odom"),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_database_path,
        rtabmap,
    ])
