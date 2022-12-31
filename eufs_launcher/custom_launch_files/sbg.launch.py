from os.path import join
from os import getenv
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = join(getenv('EUFS_MASTER'), 'config', 'sbg.yaml')

    sbg = Node(
            package='sbg_driver',
            executable='sbg_device',
            # name='sbg_device',
            parameters=[parameters]
        )

    return LaunchDescription([sbg])
