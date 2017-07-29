#!/usr/bin/env bash

# prepare ros environment
source $(dirname $0)/setup.sh

# get IP of the client if connected through SSH
if [ -n "$SSH_CONNECTION" ]; then
	client=$(IFS=" " ; set -- $SSH_CONNECTION ; echo $1)
else
	client="localhost"
fi

# set the uri of the ros master
export ROS_MASTER_URI="http://$client:11311"

# launch drivers in background
screen -dmS drivers roslaunch carote youbot.launch
