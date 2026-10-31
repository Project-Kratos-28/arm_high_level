#!/bin/bash
PORT=${1:-/dev/ttyUSB0}
BAUD=${2:-115200}
ROS_DISTRO=${ROS_DISTRO:-jazzy}

echo "=== Launching micro-ROS Agent ==="
echo "Port:       $PORT"
echo "Baudrate:   $BAUD"
echo "ROS Distro: $ROS_DISTRO"
echo ""

if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo "WARNING: Cannot read/write $PORT. Attempting temporary chmod 666 (sudo may prompt)..."
    sudo chmod 666 "$PORT" 2>/dev/null || echo "Please run './setup_permissions.sh' to fix permissions."
fi

# Source ROS 2 base and micro-ROS agent workspace
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/microros_ws/install/setup.bash" ]; then
    source "$HOME/microros_ws/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ROS 2 environment not found! Please source your ROS 2 installation."
    exit 1
fi

echo "Starting micro-ROS agent (Press Ctrl+C to stop)..."
ros2 run micro_ros_agent micro_ros_agent serial --dev "$PORT" -b "$BAUD" -v 4
