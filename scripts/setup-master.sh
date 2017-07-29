#!/usr/bin/env bash

# prepare ros environment
source $(dirname $0)/setup-environment.sh

# start the ros master
roscore
