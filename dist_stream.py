import visdom
import VL53L1X
import time
from collections import deque
import numpy as np

# Connect to visdom server
vis = visdom.Visdom()

# Create a VL53L1X distance sensor object
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()
tof.start_ranging(3)

start = time.time()
ts = deque(maxlen=100)
ds = deque(maxlen=100)
try:
    while True:
        depth = tof.get_distance() / 10.0
        ds.append(depth)
        ts.append(time.time() - start)
        vis.text(str(ds[-1]), 1)
        vis.line(X=np.array(ts), 
                 Y=np.array(ds), 
                 update='append', win=2)
        time.sleep(0.1)
except KeyboardInterrupt:
    tof.stop_ranging()


