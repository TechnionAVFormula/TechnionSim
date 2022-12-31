#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configuration variables
    base_frame = LaunchConfiguration('base_frame')
    camera_model = LaunchConfiguration('camera_model')
    camera_name = LaunchConfiguration('camera_name')
    config = LaunchConfiguration('config')
    node_name = LaunchConfiguration('node_name')
    svo_path = LaunchConfiguration('svo_path')

    # Declare the launch arguments
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Name of the base link.'
    )

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='The model of the camera. Using a wrong camera model can '
                    'disable camera features.',
        choices=['zed', 'zedm', 'zed2', 'zed2i'],
    )

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value='zed2i',
        description='The name of the camera. It can be different from the '
                    'camera model and it will be used as node `namespace`.'
    )

    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='camera',
        description='The name of the zed_wrapper node. All the topics will '
                    'have the same prefix: `/<camera_name>/<node_name>/`'
    )

    declare_config_cmd = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(
            os.environ['EUFS_MASTER'],
            'config',
            'car.yaml'
        ),
        description='Path to the config file.'
    )

    declare_svo_path_cmd = DeclareLaunchArgument(
        'svo_path',
        default_value='',
        description='Path to an input SVO file. Note: overrides the parameter '
                    '`general.svo_file` in `common.yaml`.'
    )

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        namespace='',
        name=node_name,
        output='screen',
        parameters=[
            # YAML files
            config,  # Common parameters
            # Overriding
            {
                'general.camera_name': camera_name,
                'general.camera_model': camera_model,
                'general.svo_file': svo_path,
                'pos_tracking.base_frame': base_frame
            }
        ]
    )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_config_cmd)
    ld.add_action(declare_node_name_cmd)
    ld.add_action(declare_svo_path_cmd)
    ld.add_action(zed_wrapper_node)

    return ld
