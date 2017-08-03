#!/usr/bin/env bash

# prepare ros environment
source $(dirname $0)/setup-environment.sh

# connect to the robot ros master
uri=($(roscd carote/launch; grep "uri/robot" "setup.launch"))
uri=${uri[3]//\"}
uri=${uri#value\=}
export ROS_MASTER_URI="$uri"

# start camera and target detection
roslaunch carote controller.launch
