#!/usr/bin/env bash

# launch the ros master in background
screen -dmS master $(dirname $0)/setup-master.sh
