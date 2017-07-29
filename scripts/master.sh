#!/usr/bin/env bash

# launch roscore in background
screen -dmS master $(dirname $0)/setup-master.sh

sleep 0.5

# launch camera and target detection in background
screen -dmS target $(dirname $0)/setup-target.sh
