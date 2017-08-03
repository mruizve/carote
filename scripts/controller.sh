#!/usr/bin/env bash

# launch controller in background
screen -dmS controller $(dirname $0)/setup-controller.sh

# prepare ros environment
source $(dirname $0)/setup-environment.sh

# launch dynamic reconfigure (robot master)
uri=($(roscd carote/launch; grep "uri/robot" "setup.launch"))
uri=${uri[3]//\"}
uri=${uri#value\=}
export ROS_MASTER_URI="$uri"
#rosrun rqt_reconfigure rqt_reconfigure
