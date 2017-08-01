#!/usr/bin/env bash

# prepare ros environment
source $(dirname $0)/setup-environment.sh

# connect to the robot ros master
uri=($(grep "uri/robot" "$(dirname $0)/../launch/setup.launch"))
uri=${uri[3]//\"}
uri=${uri#value\=}
export ROS_MASTER_URI="$uri"

# start camera and target detection
roslaunch carote controller.launch
