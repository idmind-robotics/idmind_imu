import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    def_imu_cfg = os.path.join(get_package_share_directory('idmind_imu'), 'config', 'idmind_imu.yaml')
    imu_cfg = DeclareLaunchArgument(name="imu_cfg", default_value=def_imu_cfg, description="Configuration file for IMU")
    ld.add_action(imu_cfg)
    
    imu_node = Node(
        package='idmind_imu',
        executable='imu_brick_node',
        name='idmind_imu',
        output='screen',
        emulate_tty=True,
        parameters = [LaunchConfiguration('imu_cfg')],
        remappings = [],
    )
    ld.add_action(imu_node)

    return ld
