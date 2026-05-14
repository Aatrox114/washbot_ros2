from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0"
    )

    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200"
    )

    config_file = PathJoinSubstitution([
        FindPackageShare("washbot_base_driver"),
        "config",
        "base_driver.yaml"
    ])

    base_driver_node = Node(
        package="washbot_base_driver",
        executable="base_driver_node",
        name="base_driver_node",
        output="screen",
        parameters=[
            config_file,
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baudrate": LaunchConfiguration("baudrate"),
            }
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        base_driver_node
    ])
