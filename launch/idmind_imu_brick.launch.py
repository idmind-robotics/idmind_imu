import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Robot name, used to namespace the node (and its topics/services).
    robot_name = DeclareLaunchArgument(
        name='robot_name', default_value='',
        description='Name of this robot, used for namespacing'
    )
    ld.add_action(robot_name)

    # Declared for a uniform bringup API across the HardwareSDK launch files; the node
    # does not consume it yet.
    simulation = DeclareLaunchArgument(
        name='simulation', default_value='False',
        description='Is the robot running in simulation?'
    )
    ld.add_action(simulation)

    def_imu_cfg = os.path.join(
        get_package_share_directory('idmind_imu'), 'config', 'idmind_imu.yaml'
    )
    imu_cfg = DeclareLaunchArgument(
        name='imu_cfg', default_value=def_imu_cfg,
        description='Configuration file for IMU'
    )
    ld.add_action(imu_cfg)

    imu_node = Node(
        package='idmind_imu',
        executable='imu_brick_node_v2',
        name='idmind_imu',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('imu_cfg')],
        remappings=[],
    )
    ld.add_action(imu_node)

    return ld
