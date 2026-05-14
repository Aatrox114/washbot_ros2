from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("washbot_description")
    default_world_file = "/home/yuandong/ros2_ws/src/carwash_sim/worlds/carwash.sdf"
    xacro_file = "/home/yuandong/ros2_ws/src/washbot_description/urdf/washbot.urdf.xacro"
    urdf_tmp = "/tmp/washbot.urdf"

    world_name = "carwash_sp"
    robot_name = "washbot"

    bridge_config = os.path.join(pkg_share, "config", "bridge.yaml")

    world_file = LaunchConfiguration("world_file")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    start_slam = LaunchConfiguration("start_slam")

    declare_world_file = DeclareLaunchArgument(
        "world_file",
        default_value=default_world_file
    )

    declare_spawn_x = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.2"
    )

    declare_spawn_y = DeclareLaunchArgument(
        "spawn_y",
        default_value="-6"
    )

    declare_spawn_z = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.2"
    )

    declare_start_slam = DeclareLaunchArgument(
        "start_slam",
        default_value="true"
    )

    # 1) 启动 Gazebo 场景
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": world_file
        }.items()
    )

    # 2) 先把 xacro 转成临时 urdf 文件，供 create 使用
    generate_urdf = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            f"source /opt/ros/humble/setup.bash && "
            f"xacro {xacro_file} > {urdf_tmp}"
        ],
        output="screen"
    )

    # 3) robot_state_publisher：负责 base_link / lidar_link / base_footprint 等静态 TF
    robot_description = Command(["xacro ", xacro_file])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True}
        ]
    )

    # 4) 补一条 scan frame 对应的静态 TF
    scan_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="scan_frame_tf_pub",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "lidar_link",
            "washbot/base_footprint/lidar_sensor"
        ]
    )

    # 5) 等 Gazebo 稍微稳定后，再生成机器人

    # 4.5) 补 RGB-D 相机消息 frame_id 对应的静态 TF
    # RTAB-Map 报错需要这个 frame:
    # washbot/base_footprint/rgbd_camera
    rgbd_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rgbd_frame_tf_pub",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "rgbd_camera_optical_frame",
            "washbot/base_footprint/rgbd_camera"
        ]
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world", world_name,
                    "-file", urdf_tmp,
                    "-name", robot_name,
                    "-x", spawn_x,
                    "-y", spawn_y,
                    "-z", spawn_z,
                ],
            )
        ]
    )

    # 6) 再延迟一点启动 bridge
    bridge = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                parameters=[{"config_file": bridge_config}],
            )
        ]
    )

    # 7) 最后再启动 slam_toolbox，避免 clock / tf / scan 还没稳定
    slam = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("slam_toolbox"),
                        "launch",
                        "online_async_launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true"
                }.items(),
                condition=IfCondition(start_slam)
            )
        ]
    )

    return LaunchDescription([
        rgbd_frame_tf,
        declare_world_file,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
    DeclareLaunchArgument(
        "world_file",
        default_value=default_world_file,
        description="Gazebo world file"
    ),

    declare_start_slam,
    gazebo,
    generate_urdf,
    rsp,
    scan_frame_tf,
    spawn_robot,
    bridge,
    slam,
])
