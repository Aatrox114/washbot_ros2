from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")
    approx_sync = LaunchConfiguration("approx_sync")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock"
    )

    declare_database_path = DeclareLaunchArgument(
        "database_path",
        default_value="/home/yuandong/ros2_ws/maps/rtabmap/carwash_rgbd.db",
        description="RTAB-Map database path"
    )

    declare_approx_sync = DeclareLaunchArgument(
        "approx_sync",
        default_value="true",
        description="Use approximate synchronization for RGB-D topics"
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,

                # 坐标系
                "frame_id": "base_footprint",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "odom_topic": "/odom",

                # RGB-D 输入
                "subscribe_rgb": True,
                "subscribe_depth": True,
                "subscribe_rgbd": False,
                "subscribe_stereo": False,

                # 先不要吃 3D 雷达，避免把地面扫成障碍物
                "subscribe_scan": False,
                "subscribe_scan_cloud": False,
                "subscribe_odom_info": False,

                # 同步
                "approx_sync": approx_sync,
                "sync_queue_size": 50,

                # 数据库
                "database_path": database_path,
                "map_always_update": True,

                # RTAB-Map 内部参数要用字符串
                # 用 RGB-D 的深度几何做 ICP 注册，避免纯视觉在 Gazebo 低纹理环境下失效
                "Reg/Strategy": "1",
                "Reg/Force3DoF": "true",
                "Optimizer/Slam2D": "true",

                # 用深度图生成 2D 栅格
                "Grid/Sensor": "1",
                "Grid/RangeMax": "6.0",
                "Grid/CellSize": "0.05",
                "Grid/NormalsSegmentation": "true",
                "Grid/NormalK": "20",
                "Grid/MaxGroundAngle": "0.52",
                "Grid/MaxGroundHeight": "0.10",
                "Grid/MaxObstacleHeight": "2.0",
                "Grid/ClusterRadius": "0.10",
                "Grid/MinClusterSize": "20",

                # ICP 参数
                "Icp/MaxCorrespondenceDistance": "0.2",
                "Icp/VoxelSize": "0.05",
                "Icp/Iterations": "30",
            }
        ],
        remappings=[
            ("rgb/image", "/camera/rgb/image_raw"),
            ("depth/image", "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/rgb/camera_info"),
            ("odom", "/odom"),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_database_path,
        declare_approx_sync,
        rtabmap,
    ])
