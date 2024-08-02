#!/bin/bash

# Execute bash inside the Docker container and source ROS 2 setup files
docker exec -i simple_amr_project bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch simple_amr_sim gz.launch.py" &

# Wait for a few seconds to ensure the first command has started
sleep 5

# Run the second command with parameters provided through shell
docker exec -i simple_amr_project bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch amr_control controller.launch.py"

