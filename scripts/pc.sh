#!/usr/bin/env bash

# launch camera and target detection in background
screen -dmS target $(dirname $0)/setup-target.sh
