#!/usr/bin/env bash
# Menjalankan arena tiruan + RViz. Butuh ROS 2 (Jazzy). Tanpa Gazebo, tanpa GPU.
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
python3 "$HERE/arena_sim.py" "$@" &
SIM=$!
trap 'kill $SIM 2>/dev/null' EXIT
rviz2 -d "$HERE/arena.rviz"
