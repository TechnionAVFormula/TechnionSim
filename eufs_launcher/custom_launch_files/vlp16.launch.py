import os

from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node


def generate_launch_description():
    # Velodyne config
    config = os.path.join(
        os.environ['EUFS_MASTER'],
        'config',
        'car.yaml'
    )

    velodyne_driver = Node(
        name='velodyne_driver_node',
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[config]
    )

    velodyne_convert = Node(
        name='velodyne_convert_node',
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        output='both',
        parameters=[ParameterFile(param_file=config, allow_substs=True)]
    )

    return LaunchDescription([
        velodyne_driver,
        velodyne_convert
    ])
