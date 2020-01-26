import multiprocessing
import visdom
import numpy as np
import time
import sys

import picamera
import picamera.array

import VL53L1X
from collections import deque

# Connect to visdom server
vis = visdom.Visdom()

start = time.time()

def camWorker(q):
    with picamera.PiCamera(resolution='320x240', framerate=10) as camera:
        camera.rotation=180
        with picamera.array.PiRGBArray(camera) as output:
            while True:
                output.truncate(0)
                camera.capture(output, 'rgb', use_video_port=True)
                t = time.time() - start
                q.put(('cam', t, output.array))
                vis.image(np.moveaxis(output.array, 2, 0), win=0)

# Create a VL53L1X distance sensor object
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()
tof.start_ranging(3)
def depthWorker(q):
    ts = deque(maxlen=100)
    ds = deque(maxlen=100)
    while True:
        depth = tof.get_distance() / 10.0
        t = time.time() - start
        q.put(('depth', t, depth))
        ds.append(depth)
        ts.append(t)
        vis.text(str(depth), win=1)
        vis.line(X=np.array(ts), 
                 Y=np.array(ds), 
                 update='append', win=2)

queue = multiprocessing.Queue()
pcam = multiprocessing.Process(target = camWorker, args=(queue,))
pcam.start()
pdepth = multiprocessing.Process(target = depthWorker, args=(queue,))
pdepth.start()
try:
    while True:
        (key, t, value) = queue.get()
        #if (key == 'cam'):
        #    print(value.shape)
        if (key == 'depth'):
            print(value)
        time.sleep(0.05)
except e:
    print(e)
    tof.stop_ranging()
    pcam.terminate()
    pdepth.terminate()
