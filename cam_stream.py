import picamera
import picamera.array
import visdom
import numpy as np
import time
import sys


# Connect to visdom server
vis = visdom.Visdom()

with picamera.PiCamera(resolution='320x240', framerate=30) as camera:
    camera.rotation=180
    with picamera.array.PiRGBArray(camera) as output:
        while True:
            output.truncate(0)
            camera.capture(output, 'rgb', use_video_port=True)
            vis.image(np.moveaxis(output.array, 2, 0), 0)

