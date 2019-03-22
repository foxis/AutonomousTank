#!/bin/bash

cd /home/pi/AutonomousTank/src/node
python3 -m EasyVision.bin.server $HOSTNAME builder_calib.json -H `hostname -I`