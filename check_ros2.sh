#!/bin/bash
# ROS 2 Discovery Diagnostic Script

echo "=== ROS 2 Discovery Diagnostics ==="
echo ""

# Check ROS 2 environment
echo "1. Checking ROS 2 environment..."
if command -v ros2 &> /dev/null; then
    echo "   ✓ ros2 command found"
    ros2 --version
else
    echo "   ✗ ros2 command NOT found - ROS 2 not in PATH"
    echo "   Try: source /opt/ros/humble/setup.bash (or your ROS 2 install path)"
    exit 1
fi

echo ""
echo "2. Checking ROS_DOMAIN_ID..."
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "   ROS_DOMAIN_ID not set (using default: 0)"
else
    echo "   ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

echo ""
echo "3. Checking for running nodes..."
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    echo "   ✗ No nodes found"
    echo "   Make sure you've launched: ros2 launch morphing_airfoil airfoil.launch.py"
else
    echo "   ✓ Found nodes:"
    echo "$NODES" | sed 's/^/     /'
fi

echo ""
echo "4. Checking for topics..."
TOPICS=$(ros2 topic list 2>/dev/null)
if [ -z "$TOPICS" ]; then
    echo "   ✗ No topics found"
else
    echo "   ✓ Found topics:"
    echo "$TOPICS" | sed 's/^/     /'
fi

echo ""
echo "5. Checking topic info for /sensors/lift..."
ros2 topic info /sensors/lift 2>/dev/null || echo "   ✗ Topic /sensors/lift does not exist"

echo ""
echo "=== Diagnostic Complete ==="
echo ""
echo "If nodes/topics are missing:"
echo "  1. Make sure ROS 2 is sourced in BOTH terminals"
echo "  2. Check ROS_DOMAIN_ID matches in both terminals"
echo "  3. Wait a few seconds after launching - ROS 2 discovery takes time"
echo "  4. Check launch output for errors"


