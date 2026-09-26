#!/usr/bin/env bash
set -euo pipefail

sudo mkdir -p /opt/overlay_ws/{src,build,install,log} /tmp/.ccache
sudo chown "$(id -u):$(id -g)" /opt/overlay_ws /opt/overlay_ws/src
sudo chown -R "$(id -u):$(id -g)" /opt/overlay_ws/{build,install,log} /tmp/.ccache
git config --global --add safe.directory /opt/overlay_ws/src/turtlebot_flatland

if [ ! -e /opt/overlay_ws/src/flatland ]; then
    git clone --branch ros2 https://github.com/avidbots/flatland.git /opt/overlay_ws/src/flatland
fi
git config --global --add safe.directory /opt/overlay_ws/src/flatland

rosdep update --rosdistro "$ROS_DISTRO"
sudo apt-get update
rosdep install --from-paths /opt/overlay_ws/src --ignore-src --rosdistro "$ROS_DISTRO" -y