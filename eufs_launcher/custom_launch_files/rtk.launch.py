"""Launch the ublox gps node with c94-m8p configuration."""

import os

import launch
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(os.environ["EUFS_MASTER"], 'config', 'rtk.yaml')
    ublox_gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[config])

    return launch.LaunchDescription([
        ublox_gps_node,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=ublox_gps_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
