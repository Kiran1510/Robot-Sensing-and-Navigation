from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    #declare the serial port argument /dev/ttyUSB0 or /dev/pts/2
    port_arg = DeclareLaunchArgument(
        'port',
        description='Serial port connected to the VectorNav IMU'
    )

    #launch VN-100 driver node
    vn_driver_node = Node(
        package='vn_driver',
        executable='vn_driver',
        name='vn_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baudrate': 115200},
            {'sampling_rate': 40.0}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        port_arg,
        vn_driver_node
    ])
