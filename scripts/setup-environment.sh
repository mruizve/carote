#!/usr/bin/env bash

# prepare ros environment
source /opt/ros/indigo/setup.bash

if [ -x ~/catkin_ws/devel/setup.bash ]; then
	source ~/catkin_ws/devel/setup.bash
fi

if [ -x ~/workspace/ros/devel/setup.sh ]; then
	source ~/workspace/ros/devel/setup.sh
fi
