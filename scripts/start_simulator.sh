#!/bin/bash

# Source the necessary setup scripts
source ~/.bashrc
source /opt/ros/galactic/setup.bash
cd $EUFS_MASTER
source install/setup.bash
ros2 launch eufs_launcher eufs_launcher.launch.py