#!/bin/bash

cd ../..
colcon build --packages-select republisher_node

. install/setup.bash
ros2 run republisher_node republisher_node