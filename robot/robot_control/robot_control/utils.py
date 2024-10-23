from pylx16a.lx16a import LX16A
import time
import threading
import math

LX16A.initialize('/dev/ttyUSB0')

servos = {
            1: {"servo": LX16A(1), "min_angle": 0, "max_angle": 240, "default": 123},
            2: {"servo": LX16A(2), "min_angle": 0, "max_angle": 240, "default": 180},
            3: {"servo": LX16A(3), "min_angle": 0, "max_angle": 240, "default": 115},
            4: {"servo": LX16A(4), "min_angle": 0, "max_angle": 240, "default": 118},
            5: {"servo": LX16A(5), "min_angle": 0, "max_angle": 240, "default": 190},
            6: {"servo": LX16A(6), "min_angle": 0, "max_angle": 240, "default": 130},
            7: {"servo": LX16A(7), "min_angle": 0, "max_angle": 240, "default": 120},
            8: {"servo": LX16A(8), "min_angle": 0, "max_angle": 240, "default": 50},
            9: {"servo": LX16A(9), "min_angle": 0, "max_angle": 240, "default": 110},
            10: {"servo": LX16A(10), "min_angle": 0, "max_angle": 240, "default": 118},
            11: {"servo": LX16A(11), "min_angle": 0, "max_angle": 240, "default": 50},
            12: {"servo": LX16A(12), "min_angle": 0, "max_angle": 240, "default": 125},
        }

delays = { 
            "front": {"in_a": 20, "in_b": 16, "pwm": 12},
            "rear": {"in_a": 26, "in_b": 19, "pwm": 13}
        }

def get_servo(id):
    return servos[id]["servo"]

def get_curent_position(id):
    angle = get_servo_default(id)
    time.sleep(0.05)
    angle = math.ceil(get_servo(id).get_physical_angle())
    return angle

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
    time.sleep(delay)
    
def top_left_up():    
    servo_id = 3
    smooth_move(servo=get_servo(servo_id), start_angle=(get_servo_default(servo_id)), target_angle=(get_servo_default(servo_id) - 25), delay=0.025)
    servo_id = 2
    move_servo(servo_id, get_servo_default(servo_id) + 25)
    servo_id = 1
    move_servo(servo_id, get_servo_default(servo_id) - 30)       
    servo_id = 2
    move_servo(servo_id, get_servo_default(servo_id)) 
    servo_id = 3
    move_servo(servo_id, get_servo_default(servo_id) + 10)
    

def top_right_up():
    servo_id = 6
    smooth_move(servo=get_servo(servo_id), start_angle=(get_servo_default(servo_id)), target_angle=(get_servo_default(servo_id) - 25), delay=0.025)
    servo_id = 5
    move_servo(servo_id, get_servo_default(servo_id) + 10)
    servo_id = 4
    move_servo(servo_id, get_servo_default(servo_id) + 30)       
    servo_id = 5
    move_servo(servo_id, get_servo_default(servo_id)) 
    servo_id = 6
    move_servo(servo_id, get_servo_default(servo_id) + 10)
    
def bot_left_up():
    servo_id = 9
    smooth_move(servo=get_servo(servo_id), start_angle=(get_servo_default(servo_id)), target_angle=(get_servo_default(servo_id) + 25), delay=0.025)
    servo_id = 8
    move_servo(servo_id, get_servo_default(servo_id) - 10)
    servo_id = 7
    move_servo(servo_id, get_servo_default(servo_id) - 20)       
    servo_id = 8
    move_servo(servo_id, get_servo_default(servo_id)) 
    servo_id = 9
    move_servo(servo_id, get_servo_default(servo_id) - 10)

def bot_right_up():
    servo_id = 12
    smooth_move(servo=get_servo(servo_id), start_angle=(get_servo_default(servo_id)), target_angle=(get_servo_default(servo_id) + 25), delay=0.025)
    servo_id = 11
    move_servo(servo_id, get_servo_default(servo_id) - 10)
    servo_id = 10
    move_servo(servo_id, get_servo_default(servo_id) + 20)       
    servo_id = 11
    move_servo(servo_id, get_servo_default(servo_id)) 
    servo_id = 12
    move_servo(servo_id, get_servo_default(servo_id) - 10)
    
def top_left_down():    
    servo_id = 3
    smooth_move(servo=get_servo(servo_id), start_angle=(get_servo_default(servo_id)), target_angle=(get_servo_default(servo_id) - 25), delay=0.025)
    servo_id = 2
    move_servo(servo_id, get_servo_default(servo_id) + 25)
    servo_id = 1
    move_servo(servo_id, get_servo_default(servo_id) + 30)       
    servo_id = 2
    move_servo(servo_id, get_servo_default(servo_id)) 
    servo_id = 3
    move_servo(servo_id, get_servo_default(servo_id) + 10)
    

def top_right_down():
    servo_id = 6
    smooth_move(servo=get_servo(servo_id), start_angle=(get_servo_default(servo_id)), target_angle=(get_servo_default(servo_id) - 25), delay=0.025)
    servo_id = 5
    move_servo(servo_id, get_servo_default(servo_id) + 10)
    servo_id = 4
    move_servo(servo_id, get_servo_default(servo_id) - 30)       
    servo_id = 5
    move_servo(servo_id, get_servo_default(servo_id)) 
    servo_id = 6
    move_servo(servo_id, get_servo_default(servo_id) + 10)
    
def bot_left_down():
    servo_id = 9
    smooth_move(servo=get_servo(servo_id), start_angle=(get_servo_default(servo_id)), target_angle=(get_servo_default(servo_id) + 25), delay=0.025)
    servo_id = 8
    move_servo(servo_id, get_servo_default(servo_id) - 10)
    servo_id = 7
    move_servo(servo_id, get_servo_default(servo_id) + 30)       
    servo_id = 8
    move_servo(servo_id, get_servo_default(servo_id)) 
    servo_id = 9
    move_servo(servo_id, get_servo_default(servo_id) - 10)

def bot_right_down():
    servo_id = 12
    smooth_move(servo=get_servo(servo_id), start_angle=(get_servo_default(servo_id)), target_angle=(get_servo_default(servo_id) + 25), delay=0.025)
    servo_id = 11
    move_servo(servo_id, get_servo_default(servo_id) - 10)
    servo_id = 10
    move_servo(servo_id, get_servo_default(servo_id) - 30)       
    servo_id = 11
    move_servo(servo_id, get_servo_default(servo_id)) 
    servo_id = 12
    move_servo(servo_id, get_servo_default(servo_id) - 10)
    