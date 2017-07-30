#!/usr/bin/env bash

cd $(dirname $0)/../build;
make -j4
error=$?
cd -

if [ "$error" -eq "0" ]; then
	# do something...
fi
