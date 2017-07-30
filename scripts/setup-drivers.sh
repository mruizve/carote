#!/usr/bin/env bash

# prepare ros environment
source "$(dirname $0)/setup-environment.sh"

# launch drivers
roslaunch carote youbot.launch
