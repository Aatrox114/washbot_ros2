from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("washbot_decision")

    default_config = os.path.join(
        pkg_share,
        "config",
        "wash_task.yaml"
    )

    config_file = LaunchConfiguration("config_file")

    declare_config = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Wash task config yaml file"
    )

    decision_node = Node(
        package="washbot_decision",
        executable="rect_patrol_node",
        name="washbot_rect_patrol",
        output="screen",
        parameters=[
            {"config_file": config_file}
        ]
    )

    return LaunchDescription([
        declare_config,
        decision_node
    ])
