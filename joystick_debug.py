import os
import pygame as pg
import time
# pygame setup
os.environ["SDL_VIDEODRIVER"] = "dummy"
pg.init()
pg.joystick.init()
joystick = pg.joystick.Joystick(0)
joystick.init()
print("Joystick online:", joystick.get_name())
while True:
    pg.event.pump()
    ax_vals = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    print(ax_vals)
    time.sleep(0.1)
