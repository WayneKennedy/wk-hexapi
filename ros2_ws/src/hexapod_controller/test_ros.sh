#!/bin/bash
# ROS 2 Controller Test
# Run inside Docker container: docker-compose run --rm dev ./test_ros.sh

set -e

echo "=========================================="
echo "Hexapod ROS 2 Controller Test"
echo "=========================================="

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build the package
echo ""
echo ">>> Building hexapod_controller..."
cd /ros2_ws
colcon build --packages-select hexapod_controller --symlink-install
source install/setup.bash

# Start controller in background
echo ""
echo ">>> Starting hexapod_controller node..."
ros2 run hexapod_controller controller &
CONTROLLER_PID=$!
sleep 2

# Cleanup function
cleanup() {
    echo ""
    echo ">>> Stopping controller..."
    kill $CONTROLLER_PID 2>/dev/null || true
    wait $CONTROLLER_PID 2>/dev/null || true
}
trap cleanup EXIT

# Initialize (home -> stand)
echo ""
echo ">>> Calling /hexapod/initialize service..."
ros2 service call /hexapod/initialize std_srvs/srv/Trigger

sleep 2

# Walk forward ~150mm continuously
# At 25mm/cycle and ~0.5s cycle time, need ~3s of continuous commands
echo ""
echo ">>> Walking FORWARD (~150mm continuous)..."
timeout 3.5s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    --rate 10 || true

sleep 1

# Walk backward ~150mm continuously
echo ""
echo ">>> Walking BACKWARD (~150mm continuous)..."
timeout 3.5s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    --rate 10 || true

sleep 1

# Home before relax
echo ""
echo ">>> Sending HOME command..."
ros2 topic pub --once /pose_command std_msgs/msg/String "{data: 'home'}"

sleep 1

# Relax
echo ""
echo ">>> Sending RELAX command..."
ros2 topic pub --once /pose_command std_msgs/msg/String "{data: 'relax'}"

sleep 1

echo ""
echo "=== TEST COMPLETE ==="
