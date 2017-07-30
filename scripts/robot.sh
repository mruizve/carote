#!/usr/bin/env bash

# launch youbot drivers in background
screen -dmS drivers $(dirname $0)/setup-drivers.sh
