#!/usr/bin/env bash

cd $(dirname $0)/build;
make -j4
cd -

roslaunch carote controller.launch
