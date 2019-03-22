#!/bin/bash

cd /home/pi/AutonomousTank/node
python3 -m EasyVision.bin.server $HOSTNAME builder.json -H `hostname -I`