FROM osrf/ros:jazzy-desktop

# Set the working directory
WORKDIR /app

# Update the system and install necessary packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y python3-pip python3-venv

# Create and set up a virtual environment
RUN python3 -m venv /app/.venv
RUN echo "source /opt/ros/jazzy/setup.bash" >> /app/.venv/bin/activate
RUN /app/.venv/bin/pip install flask pylx16a pyserial numpy PyYAML opencv-python

# Copy your application files into the container
COPY ./robot /app/robot
COPY ./controller /app/controller

# Source ROS 2 setup and build the ROS 2 workspace
RUN /bin/bash -c "source /app/.venv/bin/activate && source /opt/ros/jazzy/setup.bash && cd /app/robot && colcon build"

# Expose the required port
EXPOSE 5000

ENV ROS_DOMAIN_ID=1

# Command to run your ROS 2 node
CMD ["/bin/bash", "-c", "source /app/.venv/bin/activate && source /app/robot/install/setup.bash && ros2 run robot_control ros2_controller"]
