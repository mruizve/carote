#!/usr/bin/env bash

# launch camera and target detection in background
screen -dmS target $(dirname $0)/setup-target.sh

# wait for the ros core to be ready
sleep 1

# launch operator node in background
screen -dmS operator $(dirname $0)/setup-operator.sh
