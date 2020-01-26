import multiprocessing
import visdom
import numpy as np
import time
import sys
import tables
import cv2

# For camera
import picamera
import picamera.array

# For depth sensor
import VL53L1X
from collections import deque

# Connect to visdom server
vis = visdom.Visdom()
start = time.time()

#
# Create a VL53L1X distance sensor object
#
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()
tof.start_ranging(3)

running = multiprocessing.Value('i', 1)

def camWorker(q, running):
    with picamera.PiCamera(resolution='320x240', framerate=2) as camera:
        camera.rotation=180
        with picamera.array.PiRGBArray(camera) as output:
            while bool(running.value):
                output.truncate(0)
                camera.capture(output, 'rgb', use_video_port=True)
                t = time.time() - start
                outdata = np.moveaxis(output.array, 2, 0)
                q.put(('cam', t, outdata))
                vis.image(outdata, win=0)

def depthWorker(q, running):
    ts = deque(maxlen=30)
    ds = deque(maxlen=30)
    while bool(running.value):
        depth = tof.get_distance() / 10.0
        t = time.time() - start
        q.put(('depth', t, depth))
        ds.append(depth)
        ts.append(t)
        vis.text(str(depth), win=1)
        vis.line(X=np.array(ts), 
                 Y=np.array(ds), 
                 update='append', win=2)


def fileWorker(q, running):
    while bool(running.value):
        (key, t, value) = queue.get()
        if (key == 'cam'):
            array_camtime.append(np.array([[t]]))
            array_camdata.append(np.expand_dims(value, 0))
        if (key == 'depth'):
            array_depthtime.append(np.array([[t]]))
            array_depthdata.append([[value]])
        time.sleep(0.05)

# Create output files
timestamp = time.strftime("%Y%m%d-%H%M%S")
f = tables.open_file('data/' + timestamp + '.h5', mode='w')
array_camdata = f.create_earray(f.root, 'camdata', tables.Float32Atom(), (0, 3, 240, 320))
array_camtime = f.create_earray(f.root, 'camtime', tables.Float32Atom(), (0, 1))
array_depthdata = f.create_earray(f.root, 'depthdata', tables.Float32Atom(), (0, 1))
array_depthtime = f.create_earray(f.root, 'depthtime', tables.Float32Atom(), (0, 1))

queue = multiprocessing.Queue()
pcam = multiprocessing.Process(target = camWorker, args=(queue,running,))
pcam.start()
pdepth = multiprocessing.Process(target = depthWorker, args=(queue,running,))
pdepth.start()
pfile = multiprocessing.Process(target = fileWorker, args=(queue,running,))
pfile.start()

try:
    input()
except Exception as e:
    print(e)
finally:
    print("Shutting down processes...")
    running.value = 0
    pcam.join()
    pdepth.join()
    pfile.join()
    tof.stop_ranging()
    f.close()
