from pylx16a.lx16a import *

"""
1 30.0 156.0
2 144.0 210.0
3 36.0 84.0
4 79.2 207.6
5 144.0 210.0
6 42.0 84.0
7 84.0 210.0
8 32.400000000000006 102.0
9 187.20000000000002 240.0
10 30.0 156.0
11 30.0 92.4
12 153.6 240.0

"""

# group FL 
servo1 = LX16A(1)
servo1.set_angle_limits(125, 650)
servo2 = LX16A(2)
servo2.set_angle_limits(600, 875)
servo3 = LX16A(3)
servo3.set_angle_limits(150, 350)

# group FR
servo4 = LX16A(4)
servo4.set_angle_limits(330, 865)
servo5 = LX16A(5)
servo5.set_angle_limits(600, 875)
servo6 = LX16A(6)
servo6.set_angle_limits(175, 350)

# group RL
servo7 = LX16A(7)
servo7.set_angle_limits(350, 875)
servo8 = LX16A(8)
servo8.set_angle_limits(135, 425)
servo9 = LX16A(9)
servo9.set_angle_limits(780, 1000)

# group RR
servo10 = LX16A(10)
servo10.set_angle_limits(125, 650)
servo11 = LX16A(11)
servo11.set_angle_limits(125, 385)
servo12 = LX16A(12)
servo12.set_angle_limits(640, 1000)
