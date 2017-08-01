#!/usr/bin/env bash

# make package
cd $(dirname $0)/../build;
make -j4
error=$?
cd -

if [ "$error" -eq "0" ]; then
	# launch camera and target detection in background
	screen -dmS target $(dirname $0)/setup-target.sh
fi
