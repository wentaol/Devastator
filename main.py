import multiprocessing
from queue import Empty
import visdom
import numpy as np
import time
from datetime import timedelta
import sys
import os
import tables
import cv2
import pickle

# For camera
from jetson_camera import gstreamer_pipeline

# For depth sensor
import VL53L1X
from collections import deque


IM_W = 1280
IM_H = 720

# Writes camera data to tmqueue
def camWorker(q, running):
    with open("cameramtx.dat", "rb") as f:
        cam_dict = pickle.load(f)
    stream = gstreamer_pipeline(flip_method=2, 
                                capture_width=IM_W, 
                                capture_height=IM_H,
                                display_width=IM_W, 
                                display_height=IM_H,
                                framerate=10)
    cap = cv2.VideoCapture(stream, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print("Camera online!")
        while bool(running.value):
            ret_val, output = cap.read()
            t = time.time() - start
            output = cv2.undistort(output, cam_dict["mtx"], cam_dict["dist"], 
                                   None, cam_dict["newcameramtx"])
            #print(output.shape)
            q.put(('cam', t, output))
            outdata = cv2.resize(output, (320, 180))
            outdata = np.moveaxis(outdata, 2, 0)
            vis.image(np.flip(outdata, axis=0), win=0)
        cap.release()
        print("Camera thread terminated.")
    else:
        print("Unable to open camera")

## Writes depth sensor readings to tmqueue
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
        #vis.text(str(depth), win=1)
        vis.line(X=np.array(ts), 
                 Y=np.array(ds), 
                 update='append', win=2)
    tof.stop_ranging()
    print("Depth sensor thread terminated.")

# Consumes from tmqueue and writes output to file
def fileWorker(q, outq, running):
    # Create output files
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    if not os.path.isdir('data'):
        os.mkdir('data')
    datadir = os.path.join('data', timestamp)
    if not os.path.isdir(datadir):
        os.mkdir(datadir)
    f = tables.open_file(os.path.join(datadir, "data.h5"), mode='w')    
    #array_camdata = f.create_earray(f.root, 'camdata', tables.Float32Atom(), (0, 3, IM_H, IM_W))
    array_camtime = f.create_earray(f.root, 'camtime', tables.Float32Atom(), (0, 1))
    array_depthdata = f.create_earray(f.root, 'depthdata', tables.Float32Atom(), (0, 1))
    array_depthtime = f.create_earray(f.root, 'depthtime', tables.Float32Atom(), (0, 1))
    array_motordata = f.create_earray(f.root, 'motordata', tables.Float32Atom(), (0, 2))
    array_motortime = f.create_earray(f.root, 'motortime', tables.Float32Atom(), (0, 1))
    livetms = {}
    print("File thread online!")
    camidx = 0
    while bool(running.value):
        try:
            (key, t, value) = q.get(timeout=1)
            t_arr = np.array([[t]])
            if (key == 'cam'):
                array_camtime.append(t_arr)
                #array_camdata.append(np.expand_dims(value, 0))
                cv2.imwrite(os.path.join(datadir, str(camidx) + ".jpg"), value)
                camidx += 1
            elif (key == 'depth'):
                array_depthtime.append(t_arr)
                array_depthdata.append([[value]])
            elif (key == 'motor'):
                array_motortime.append(t_arr)
                array_motordata.append(np.array(value).reshape(1,2))
            elif (key == 'startcollection'):
                print('Start collection', value)
                livetms[value] = []
            elif (key == 'stopcollection'):
                print('Stop collection', value)
                outq.put(livetms[value])
                livetms.pop(value, None) 
            if key in livetms.keys():
                livetms[key].append((t, value))
        except Exception as e:
            print(e)
            pass
    print("Closing file...")
    f.close()
    print("File thread terminated.")

# Motor commanding and telemetry recording helper function
def motor(l, r, dt, tcq, tmq, stop=True):
    t = time.time() - start
    tcq.put((l,r))
    tmq.put(('motor', t, [l, r]))
    time.sleep(dt) 
    if stop:
        t2 = time.time() - start
        tcq.put((0.0, 0.0))
        tmq.put(('motor', t2, [0.0, 0.0]))   

## Gets input from joystick and writes motion commands to tcQueue
import pygame as pg
def joystickWorker(tmq, tcq, running):
    # pygame setup
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pg.init()
    pg.joystick.init()
    joystick = pg.joystick.Joystick(0)
    joystick.init()
    print("Joystick online:", joystick.get_name())
    while bool(running.value):
        if bool(ismanual.value):
            pg.event.pump()
            if joystick.get_button(8):
                ismanual.value = 0
            fw_ax = -joystick.get_axis(1)
            lr_ax = joystick.get_axis(2)
            # Convert forward and left right to motor values
            is_forward = 1 if fw_ax >= 0 else -1
            lax = max(-1.0, min(1.0, fw_ax + is_forward * lr_ax))
            rax = max(-1.0, min(1.0, fw_ax - is_forward * lr_ax))
            motor(lax, rax, 0.1, tcq, tmq, False)
        else:
            time.sleep(0.1)
    print("Joystick thread terminated.")


# Reads from tc queue and sends output to motors
import Jetson.GPIO as GPIO
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
    #pwm_l = GPIO.PWM(LE, 100)
    #pwm_r = GPIO.PWM(RE, 100)
    #pwm_l.start(0)
    #pwm_r.start(0)
    print("Motor online!")
    while bool(running.value):
        try:
            lax, rax = q.get(timeout=1.0)
        except Empty:
            # Safety stop if joystick thread isn't working
            # print("No input from joystick thread!")
            lax = 0.0
            rax = 0.0
        #print("L: %.3f, R:%.3f" % (lax, rax),
        #      end="\r", flush=True)
        # Set motor outputs
        GPIO.output(LP, lax > 0)
        GPIO.output(LN, lax < 0)
        GPIO.output(RP, rax > 0)
        GPIO.output(RN, rax < 0)
        #pwm_l.ChangeDutyCycle(int(100 * abs(lax)))
        #pwm_r.ChangeDutyCycle(int(100 * abs(rax)))
        GPIO.output(LE, abs(lax) > 0.1)
        GPIO.output(RE, abs(rax) > 0.1)
    # Turn everything off
    GPIO.output(LP, False)
    GPIO.output(LN, False)
    GPIO.output(RP, False)
    GPIO.output(RN, False)
    GPIO.output(LE, False)
    GPIO.output(RE, False)
    print("Motor control thread terminated.")


#
# Core thread routines
#

# Calibration routine
def run_calibration(tmq, tcq, outq):
    ismanual.value = 0
    # Clear telemetry buffer and start collection
    tmq.put(("startcollection", None, "depth"))
    time.sleep(1.0)
    # Execute movement routine
    tmpend = time.time() + 5.0 
    while time.time() < tmpend:
        motor(1.0, 1.0, 0.1, tcq, tmq, False)
    # Stop collecting telemetry to buffer
    tmq.put(("stopcollection", None, "depth"))
    time.sleep(1.0)
    # Perform analysis and output results
    try:
        outdata = outq.get()
        velocities = []
        for i in range(len(outdata)-1):
            dt = outdata[i+1][0] - outdata[i][0]
            dz = outdata[i][1] - outdata[i+1][1]
            velocities.append(dz / dt)
        print("Max v:", np.max(velocities))
        print("Median v:", np.median(velocities))
    except:
        print("Failed to get data from queue!")

# scan routine
def run_scan(tmq, tcq, outq):
    ismanual.value = 0
    # Clear telemetry buffer and start collection
    tmq.put(("startcollection", None, "depth"))
    time.sleep(1.0)
    # Execute movement routine 
    for i in range(10):
        motor(1.0, -1.0, 0.4, tcq, tmq, True)
        time.sleep(1.5)
    # Stop collecting telemetry to buffer
    tmq.put(("stopcollection", None, "depth"))
    time.sleep(1.0)
    # Perform analysis and output results
    try:
        outdata = outq.get()
        print("Scan output:")
        print(outdata)
    except:
        print("Failed to get data from queue!")
#
# Main Loop
#
if __name__ == "__main__":
    # Connect to visdom server
    vis = visdom.Visdom()
    start = time.time()
    running = multiprocessing.Value('i', 1)
    ismanual = multiprocessing.Value('i', 0)
    # Telemetry
    tmQueue = multiprocessing.Queue()
    livetmQueue = multiprocessing.Queue() # only activated for live collection
    pcam = multiprocessing.Process(target = camWorker, args=(tmQueue, running,))
    pcam.start()
    pdepth = multiprocessing.Process(target = depthWorker, args=(tmQueue, running,))
    pdepth.start()
    pfile = multiprocessing.Process(target = fileWorker, args=(tmQueue, livetmQueue, running,))
    pfile.start()

    # Telecommands
    tcQueue = multiprocessing.Queue()
    pmotor = multiprocessing.Process(target = motorWorker, args=(tcQueue, running,))
    pmotor.start()
    pjoy = multiprocessing.Process(target = joystickWorker, args=(tmQueue, tcQueue, running,))
    pjoy.start()

    try:
        while bool(running.value):
            kbinput = input("Input 'q' to quit:")
            if kbinput  == 'q' or kbinput =='quit':
                running.value = 0
            elif kbinput == 'calibrate':
                print('Begin calibration routine...')
                run_calibration(tmQueue, tcQueue, livetmQueue)
            elif kbinput == 'scan':
                print('Begin scan routine...')
                run_scan(tmQueue, tcQueue, livetmQueue)
            elif kbinput == 'manual start':
                print('Enabling manual joystick control...')
                ismanual.value = 1
                time.sleep(1.0)
            elif kbinput == 'manual stop':
                print('Disabling manual joystick control...')
                ismanual.value = 0
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
