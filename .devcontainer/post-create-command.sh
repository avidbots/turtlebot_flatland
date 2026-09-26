#!/usr/bin/env bash
set -euo pipefail

setup_line='if [ -f /opt/overlay_ws/install/setup.bash ]; then source /opt/overlay_ws/install/setup.bash; else source /opt/ros/$ROS_DISTRO/setup.bash; fi'
if ! grep -Fxq "$setup_line" "$HOME/.bashrc"; then
    printf '\n%s\n' "$setup_line" >> "$HOME/.bashrc"
fi