import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('alpha'),
        'config',
        'params.yaml'
        )
        
    sim_node = Node(
        package = 'alpha',
        name = 'uam_sim_node',
        executable = 'dynamics',
        parameters = [config]
    )

    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('alpha'), 'config', 'config.rviz')]
        )

    ld.add_action(sim_node)
    ld.add_action(rviz_node)
    return ld