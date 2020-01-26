# import curses and GPIO
import RPi.GPIO as GPIO
import pygame as pg
import os #added so we can shut down OK
import time #import time module

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

# pygame setup
os.environ["SDL_VIDEODRIVER"] = "dummy"
pg.init()
pg.joystick.init()
joystick = pg.joystick.Joystick(0)
joystick.init()
print(joystick.get_name())

try:
    while True:
        pg.event.pump()
        if joystick.get_button(8):
            break
        fw_ax = -joystick.get_axis(1)
        lr_ax = joystick.get_axis(3)
        # Convert forward and left right to motor values
        is_forward = 1 if fw_ax >= 0 else -1
        lax = max(-1.0, min(1.0, fw_ax + is_forward * lr_ax))
        rax = max(-1.0, min(1.0, fw_ax - is_forward * lr_ax))
        # Set motor outputs
        GPIO.output(LP, lax > 0)
        GPIO.output(LN, lax < 0)
        GPIO.output(RP, rax > 0)
        GPIO.output(RN, rax < 0)
        pwm_l.ChangeDutyCycle(int(100 * abs(lax)))
        pwm_r.ChangeDutyCycle(int(100 * abs(rax)))
        print("L: %.3f, R:%.3f" 
              % (lax, rax), 
              end="\r", flush=True)
        time.sleep(0.1)       
finally:
    pwm_l.stop()
    pwm_r.stop()
    GPIO.cleanup()
    

