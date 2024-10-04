from math import sin, cos, pi
from pylx16a.lx16a import *
import time




class Robot:
    
    def __init__(self):
        LX16A.initialize("/dev/ttyUSB0")

        # Front Left (FL), Front Right (FR), Rear Left (RL), Rear Right (RR)
        self.servos = {
            1: {"servo": LX16A(1), "min_angle": 30.0, "max_angle": 156.0},
            2: {"servo": LX16A(2), "min_angle": 144.0, "max_angle": 210.0},
            3: {"servo": LX16A(3), "min_angle": 36.0, "max_angle": 84.0},
            4: {"servo": LX16A(4), "min_angle": 79.2, "max_angle": 207.6},
            5: {"servo": LX16A(5), "min_angle": 144.0, "max_angle": 210.0},
            6: {"servo": LX16A(6), "min_angle": 42.0, "max_angle": 84.0},
            7: {"servo": LX16A(7), "min_angle": 84.0, "max_angle": 210.0},
            8: {"servo": LX16A(8), "min_angle": 32.4, "max_angle": 102.0},
            9: {"servo": LX16A(9), "min_angle": 187.2, "max_angle": 240.0},
            10: {"servo": LX16A(10), "min_angle": 30.0, "max_angle": 156.0},
            11: {"servo": LX16A(11), "min_angle": 30.0, "max_angle": 92.4},
            12: {"servo": LX16A(12), "min_angle": 153.6, "max_angle": 240.0},
        }

        # Set the angle limits for all servos
        for servo_data in self.servos.values():
            servo = servo_data["servo"]
            servo.set_angle_limits(servo_data["min_angle"], servo_data["max_angle"])

    def move_servo(self, servo_id, position):
        servo_data = self.servos[servo_id]
        min_angle = servo_data["min_angle"]
        max_angle = servo_data["max_angle"]

        # Ensure position is within the min/max bounds
        if position < min_angle:
            position = min_angle
        elif position > max_angle:
            position = max_angle

        servo_data["servo"].move(position)

    def move_forward(self):
        # Example: Move Front Left (1-3) -> Rear Right (10-12) -> Front Right (4-6) -> Rear Left (7-9)
        self.move_leg([1, 2, 3])
        self.move_leg([10, 11, 12])
        self.move_leg([4, 5, 6])
        self.move_leg([7, 8, 9])
        return True

    def move_leg(self, servo_ids):
        # Lift, move forward, lower, and move backward each leg
        self.lift_leg(servo_ids)
        self.move_leg_forward(servo_ids)
        self.lower_leg(servo_ids)
        self.move_leg_backward(servo_ids)

    def lift_leg(self, servo_ids):
        # Lift leg by adjusting the up/down servo (servo[0] in each leg)
        self.move_servo(servo_ids[0], self.servos[servo_ids[0]]["min_angle"])  # Lift
        time.sleep(0.2)

    def lower_leg(self, servo_ids):
        # Lower leg
        self.move_servo(servo_ids[0], self.servos[servo_ids[0]]["max_angle"])  # Lower
        time.sleep(0.2)

    def move_leg_forward(self, servo_ids):
        # Move the leg forward
        self.move_servo(servo_ids[1], self.servos[servo_ids[1]]["min_angle"])  # Move forward
        time.sleep(0.2)

    def move_leg_backward(self, servo_ids):
        # Move the leg backward to its original position
        self.move_servo(servo_ids[1], self.servos[servo_ids[1]]["max_angle"])  # Move backward
        time.sleep(0.2)