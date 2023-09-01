import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    ns = LaunchConfiguration('ns', default="")
    def_imu_cfg = os.path.join(get_package_share_directory('idmind_imu'), 'config', 'idmind_imu.yaml')
    imu_cfg = LaunchConfiguration('imu_cfg', default=def_imu_cfg)    
    
    imu_node = Node(
        package='idmind_imu',
        namespace=ns,
        executable='imu_brick_node',
        name='idmind_imu',
        output='screen',
        emulate_tty=True,
        parameters = [imu_cfg],
        remappings = [],
    )
    ld.add_action(imu_node)

    return ld