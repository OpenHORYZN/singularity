#!/bin/bash

cd ../..
colcon build --packages-select singularity

. install/setup.bash
ros2 run singularity singularity --ros-args -p "machine:=$1" 
