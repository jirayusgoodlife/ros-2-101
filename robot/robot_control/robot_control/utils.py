from pylx16a.lx16a import LX16A
import time
import threading

LX16A.initialize('/dev/ttyUSB0')

servos = {
            1: {"servo": LX16A(1), "min_angle": 30, "max_angle": 155,  "low": 90, "high": 150, "swing": [55, 120], "default": 90},
            2: {"servo": LX16A(2), "min_angle": 150, "max_angle": 210, "low": 160, "high": 190, "default": 180},
            3: {"servo": LX16A(3), "min_angle": 30, "max_angle": 150, "low": 90, "high": 40, "default": 115},
            4: {"servo": LX16A(4), "min_angle": 80, "max_angle": 200, "low": 150, "high": 80, "swing": [180, 120], "default": 150},
            5: {"servo": LX16A(5), "min_angle": 140, "max_angle": 210, "low": 160, "high": 240, "default": 190},
            6: {"servo": LX16A(6), "min_angle": 40, "max_angle": 200, "low": 100, "high": 50, "default": 130},
            7: {"servo": LX16A(7), "min_angle": 80, "max_angle": 210, "low": 155, "high": 90, "swing": [120, 180], "default": 155},
            8: {"servo": LX16A(8), "min_angle": 31, "max_angle": 120, "low": 110, "high": 60, "default": 50},
            9: {"servo": LX16A(9), "min_angle": 80, "max_angle": 170, "low": 130, "high": 160, "default": 110},
            10: {"servo": LX16A(10), "min_angle": 30, "max_angle": 170, "low": 80, "high": 150, "swing": [80, 120], "default": 80},
            11: {"servo": LX16A(11), "min_angle": 25, "max_angle": 120, "low": 90, "high": 25, "default": 50},
            12: {"servo": LX16A(12), "min_angle": 100, "max_angle": 200, "low": 150, "high": 190, "default": 125},
        }

delays = { 
            "front": {"in_a": 20, "in_b": 16, "pwm": 12},
            "rear": {"in_a": 26, "in_b": 19, "pwm": 13}
        }

def get_servo(id):
    return servos[id]["servo"]

def get_servo_default(id):
    return servos[id]["default"]

def smooth_move(servo, start_angle, target_angle, step=1, delay=0.01):
        if start_angle < target_angle:
            for angle in range(start_angle, target_angle + 1, step):
                servo.move(angle)
                time.sleep(delay)
        else:
            for angle in range(start_angle, target_angle - 1, -step):
                servo.move(angle)
                time.sleep(delay)

def move_servo(servo_id, angle, delay=0.05):
    get_servo(servo_id).move(angle)
    # time.sleep(delay)