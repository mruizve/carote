#!/usr/bin/env bash

# launch camera and target detection in background
screen -dmS target $(dirname $0)/setup-target.sh

# launch controller in background
screen -dmS controller $(dirname $0)/setup-controller.sh

# launch dynamic reconfigure (robot master)
rosscooby
rosrun rqt_reconfigure rqt_reconfigure
