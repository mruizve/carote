#!/usr/bin/env bash

# launch controller in background
screen -dmS controller $(dirname $0)/setup-controller.sh

