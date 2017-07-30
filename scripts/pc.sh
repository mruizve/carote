#!/usr/bin/env bash

# make package
cd "$(dirname $0)/../build";
make -j4
error=$?
cd -

if [ "$error" -eq "0" ]; then
	# launch camera and target detection in background
	screen -dmS target "$(dirname $0)/setup-target.sh"

	# launch dynamic reconfigure (robot master)
	uri=($(grep "uri/robot" "$(dirname $0)/../launch/setup.launch"))
	uri=${uri[3]//\"}
	uri=${uri#value\=}
	export ROS_MASTER_URI="$uri"
	rosrun rqt_reconfigure rqt_reconfigure
fi
