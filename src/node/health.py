#!/usr/local/bin/python3

import psutil
import os
from datetime import datetime

cpu = psutil.cpu_percent(interval=1, percpu=True)
temp = psutil.sensors_temperatures()['cpu0-thermal'][0].current
freq = psutil.cpu_freq().current

result = cpu + [freq, temp]

print(os.uname().nodename, datetime.now())
print(" ".join(str(i) for i in result))