from launch import LaunchDescription

import launch_ros.actions

import yaml


def generate_launch_description():

    params = yaml.load('params.yaml')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='bbr_sawtooth_bridge',
            node_executable='bridge_cpp',
            output='screen',
            parameters=[params]),
    ])
