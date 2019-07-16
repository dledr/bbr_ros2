import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

import launch_ros.actions


def generate_launch_description():

    bridge_package = get_package_share_directory('bbr_sawtooth_bridge')
    params_file_path = os.path.join(bridge_package, 'launch', 'params.yaml')
    signer_key_path = os.path.join(bridge_package, 'launch', 'signer_key.txt')
    batcher_key_path = os.path.join(bridge_package, 'launch', 'batcher_key.txt')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='bbr_sawtooth_bridge',
            node_executable='bridge_cpp',
            output='screen',
            parameters=[params_file_path],
            arguments=[[signer_key_path], [batcher_key_path]]),
    ])
