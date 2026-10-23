#!/bin/bash

# Exit immediately if a command fails
set -e

# Move to workspace
cd ~/workspaces/NDRTS-BFMC2024

python3 src/permisiuni.py

# Source the ROS 2 environment
echo "📡 Sourcing ROS 2 workspace..."
source install/setup.bash

# Launch the car logic
echo "🚗 Launching brain_launch.py..."
ros2 launch ndrts_launch brain_launch.py
