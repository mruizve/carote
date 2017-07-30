#!/usr/bin/env bash

# launch youbot drivers in background
screen -dmS drivers "$(dirname $0)/setup-drivers.sh"

# launch controller in background
screen -dmS controller "$(dirname $0)/setup-controller.sh"

