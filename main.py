import multiprocessing
from queue import Empty
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
running = multiprocessing.Value('i', 1)

# Writes camera data to tmqueue
def camWorker(q, running):
    with picamera.PiCamera(resolution='320x240', framerate=2) as camera:
        camera.rotation=180
        with picamera.array.PiRGBArray(camera) as output:
            print("Camera online!")
            while bool(running.value):
                output.truncate(0)
                camera.capture(output, 'rgb', use_video_port=True)
                t = time.time() - start
                outdata = np.moveaxis(output.array, 2, 0)
                q.put(('cam', t, outdata))
                vis.image(outdata, win=0)
            print("Camera thread terminated.")

# Writes depth sensor readings to tmqueue
def depthWorker(q, running):
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
    tof.open()
    tof.start_ranging(3)
    ts = deque(maxlen=30)
    ds = deque(maxlen=30)
    print("Depth sensor online!")
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
    tof.stop_ranging()
    print("Depth sensor thread terminated.")

# Consumes from tmqueue and writes output to file
def fileWorker(q, running):
    # Create output files
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    f = tables.open_file('data/' + timestamp + '.h5', mode='w')    
    array_camdata = f.create_earray(f.root, 'camdata', tables.Float32Atom(), (0, 3, 240, 320))
    array_camtime = f.create_earray(f.root, 'camtime', tables.Float32Atom(), (0, 1))
    array_depthdata = f.create_earray(f.root, 'depthdata', tables.Float32Atom(), (0, 1))
    array_depthtime = f.create_earray(f.root, 'depthtime', tables.Float32Atom(), (0, 1))
    array_motordata = f.create_earray(f.root, 'motordata', tables.Float32Atom(), (0, 2))
    array_motortime = f.create_earray(f.root, 'motortime', tables.Float32Atom(), (0, 1))
    while bool(running.value):
        (key, t, value) = q.get()
        t_arr = np.array([[t]]) 
        if (key == 'cam'):
            array_camtime.append(t_arr)
            array_camdata.append(np.expand_dims(value, 0))
        if (key == 'depth'):
            array_depthtime.append(t_arr)
            array_depthdata.append([[value]])
        if (key == 'motor'):
            array_motortime.append(t_arr)
            array_motordata.append(np.array(value).reshape(1,2))
        time.sleep(0.05)
    f.close()
    print("File thread terminated.")

# Gets input from joystick and writes motion commands to tcQueue
import pygame as pg
import os
def joystickWorker(tmq, tcq, running):
    # pygame setup
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pg.init()
    pg.joystick.init()
    joystick = pg.joystick.Joystick(0)
    joystick.init()
    print("Joystick online:", joystick.get_name())
    while bool(running.value):
        pg.event.pump()
        if joystick.get_button(8):
            running.value = 0
            break
        fw_ax = -joystick.get_axis(1)
        lr_ax = joystick.get_axis(3)
        # Convert forward and left right to motor values
        is_forward = 1 if fw_ax >= 0 else -1
        lax = max(-1.0, min(1.0, fw_ax + is_forward * lr_ax))
        rax = max(-1.0, min(1.0, fw_ax - is_forward * lr_ax))
        tcq.put((lax, rax))
        t = time.time() - start
        tmq.put(('motor', t, [lax, rax]))
        time.sleep(0.1)
    print("Joystick thread terminated.")

# Reads from tc queue and sends output to motors
import RPi.GPIO as GPIO
def motorWorker(q, running):
    #set GPIO numbering mode and define output pins
    LE = 18 # Brown
    RE = 22 # Violet
    LP = 11 # Yellow
    RP = 13 # Green
    LN = 16 # Orange
    RN = 15 # Blue
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LP,GPIO.OUT)
    GPIO.setup(RP,GPIO.OUT)
    GPIO.setup(LN,GPIO.OUT)
    GPIO.setup(RN,GPIO.OUT)
    GPIO.setup(LE,GPIO.OUT)
    GPIO.setup(RE,GPIO.OUT)
    pwm_l = GPIO.PWM(LE, 100)
    pwm_r = GPIO.PWM(RE, 100)
    pwm_l.start(0)
    pwm_r.start(0)
    print("Motor online!")
    while bool(running.value):
        try:
            lax, rax = q.get(timeout=1.0)
        except Empty:
            # Safety stop if joystick thread isn't working
            print("No input from joystick thread!")
            lax = 0.0
            rax = 0.0
        print("L: %.3f, R:%.3f" % (lax, rax),
              end="\r", flush=True)
        # Set motor outputs
        GPIO.output(LP, lax > 0)
        GPIO.output(LN, lax < 0)
        GPIO.output(RP, rax > 0)
        GPIO.output(RN, rax < 0)
        pwm_l.ChangeDutyCycle(int(100 * abs(lax)))
        pwm_r.ChangeDutyCycle(int(100 * abs(rax)))
    print("Motor control thread terminated.")

# Telemetry
tmQueue = multiprocessing.Queue()
pcam = multiprocessing.Process(target = camWorker, args=(tmQueue, running,))
pcam.start()
pdepth = multiprocessing.Process(target = depthWorker, args=(tmQueue, running,))
pdepth.start()
pfile = multiprocessing.Process(target = fileWorker, args=(tmQueue, running,))
pfile.start()

# Telecommands
tcQueue = multiprocessing.Queue()
pmotor = multiprocessing.Process(target = motorWorker, args=(tcQueue, running,))
pmotor.start()
pjoy = multiprocessing.Process(target = joystickWorker, args=(tmQueue, tcQueue, running,))
pjoy.start()

try:
    while bool(running.value):
        time.sleep(1.0)
except Exception as e:
    print(e)
finally:
    print("Shutting down processes...")
    running.value = 0
    pcam.join()
    pdepth.join()
    pfile.join()
    pmotor.join()
    pjoy.join()
