#!/usr/bin/env bash

# prepare ros environment
source /opt/ros/indigo/setup.bash

if [ -f ~/catkin_ws/devel/setup.bash ]; then
	source ~/catkin_ws/devel/setup.bash
fi

if [ -f ~/workspace/ros/devel/setup.sh ]; then
	source ~/workspace/ros/devel/setup.sh
fi
