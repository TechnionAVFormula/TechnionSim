FROM osrf/ros:galactic-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    tmux \
    vim \
    nano \
    python3-tk \
    python3-rosdep \
    ros-galactic-gazebo-ros \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-xacro \
    ros-galactic-ackermann-msgs \
    ros-galactic-joint-state-publisher-gui && \
    apt-get upgrade -y ros-galactic-geometry-msgs && \
    pip3 install colcon-common-extensions -U

# Instead of setting EUFS_MASTER in ~/.bashrc, use ENV to ensure it's available
ENV EUFS_MASTER=/eufs
RUN echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc

# Set the working directory to /eufs
WORKDIR /eufs

# Copy the current directory contents into the container at /eufs
COPY . .

RUN git clone https://gitlab.com/eufs/eufs_msgs.git && \
    git clone https://gitlab.com/eufs/eufs_rviz_plugins.git

# Use the ENV variable directly in the command. Ensure paths are correctly specified
RUN rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y

# Compile your ROS workspace
RUN . /opt/ros/galactic/setup.bash && \
    colcon build && \
    echo 'source /eufs/install/setup.bash' >> ~/.bashrc

# Expose the default port for ROS
EXPOSE 11311

ADD start_simulator.sh /

# open a shell as the container's entrypoint
CMD ["/bin/bash"]
