#!/usr/bin/env bash

cd $(dirname $0)/build;
make -j4
cd -

./pc.sh
