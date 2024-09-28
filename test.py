from math import sin, cos, pi
from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")

try:
    servo1 = LX16A(1)
    servo1.set_angle_limits(125, 650)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

t = 0
while True:
    servo1.move(sin(t) * 60 + 60)

    time.sleep(0.05)
    t += 0.1