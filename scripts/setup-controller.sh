#!/usr/bin/env bash

# prepare ros environment
source "$(dirname $0)/setup-environment.sh"

# start camera and target detection
roslaunch carote controller.launch
