# Use the official ROS Humble base image
FROM ros:humble-ros-base

# Set the working directory
WORKDIR /root/ros2_ws

# Copy the package source files into the container
COPY . .

# Install necessary dependencies and tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-nav2-common \
    ros-humble-nav2-util \
    ros-humble-nav2-msgs \
    ros-humble-nav2-map-server \
    ros-humble-navigation2 \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    libceres-dev \
    nano \
    wget \
    gnupg2 \
    apt-transport-https \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Clone RPLIDAR ROS 2 package
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git src/rplidar_ros

# Clone rosbridge_suite package
RUN git clone https://github.com/RobotWebTools/rosbridge_suite.git src/rosbridge_suite

# Clone RF2O package
RUN git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git src/rf2o_laser_odometry

# Clone SLAM Toolbox from the correct branch
RUN git clone -b humble https://github.com/SteveMacenski/slam_toolbox.git src/slam_toolbox

# Initialize and update rosdep
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

# Install dependencies using rosdep
RUN rosdep install --from-paths src --ignore-src -r -y || true

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source the setup script
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
