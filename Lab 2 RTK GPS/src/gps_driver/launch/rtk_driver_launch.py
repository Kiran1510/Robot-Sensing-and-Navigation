from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #declare launch argument called "port" with a default value
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port for the RTK GNSS receiver (e.g., /dev/ttyUSB0, /dev/pts/3)"
    )

    #define rtk node and pass the launch argument as parameter
    rtk_node = Node(
        package="gps_driver",
        executable="rtk_driver.py",
        name="rtk_publisher",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port")
        }]
    )

    #return complete launch description
    return LaunchDescription([
        port_arg,
        rtk_node
    ])
