from pylx16a.lx16a import *
import RPi.GPIO as GPIO
import time

class Robot:
    def __init__(self):
        # Initialize LX16A communication
        LX16A.initialize('/dev/ttyUSB0')
        self.servos = {
            1: {"servo": LX16A(1), "min_angle": 30, "max_angle": 155,  "low": 90, "high": 150, "default": 90},
            2: {"servo": LX16A(2), "min_angle": 150, "max_angle": 210, "low": 160, "high": 190, "default": 160},
            3: {"servo": LX16A(3), "min_angle": 30, "max_angle": 100, "low": 90, "high": 40, "default": 90},
            4: {"servo": LX16A(4), "min_angle": 80, "max_angle": 200, "low": 150, "high": 80, "default": 150},
            5: {"servo": LX16A(5), "min_angle": 140, "max_angle": 210, "low": 160, "high": 240, "default": 160},
            6: {"servo": LX16A(6), "min_angle": 40, "max_angle": 110, "low": 100, "high": 50, "default": 100},
            7: {"servo": LX16A(7), "min_angle": 80, "max_angle": 210, "low": 190, "high": 90, "default": 155},
            8: {"servo": LX16A(8), "min_angle": 31, "max_angle": 120, "low": 110, "high": 60, "default": 80},
            9: {"servo": LX16A(9), "min_angle": 120, "max_angle": 170, "low": 130, "high": 160, "default": 145},
            10: {"servo": LX16A(10), "min_angle": 30, "max_angle": 170, "low": 80, "high": 170, "default": 80},
            11: {"servo": LX16A(11), "min_angle": 25, "max_angle": 90, "low": 90, "high": 25, "default": 70},
            12: {"servo": LX16A(12), "min_angle": 130, "max_angle": 200, "low": 150, "high": 190, "default": 150},
        }
        
        self.prikthai = { 
            "front": {"in_a": 20, "in_b": 16, "pwm": 12},
            "rear": {"in_a": 26, "in_b": 19, "pwm": 13}
        }
        GPIO.setmode(GPIO.BCM)
        for direction in self.prikthai.values():
            GPIO.setup(direction["in_a"], GPIO.OUT)
            GPIO.setup(direction["in_b"], GPIO.OUT)
            GPIO.setup(direction["pwm"], GPIO.OUT)
        self.pwm_f = GPIO.PWM(self.prikthai["front"]["pwm"], 5000)
        self.pwm_r = GPIO.PWM(self.prikthai["rear"]["pwm"], 5000) 
        
    
    def start(self):
        self.move_servo(1, self.servos[1]["default"])
        self.move_servo(2, self.servos[2]["default"])
        self.move_servo(3, self.servos[3]["default"])
        self.move_servo(4, self.servos[4]["default"])
        self.move_servo(5, self.servos[5]["default"])
        self.move_servo(6, self.servos[6]["default"])
        self.move_servo(7, self.servos[7]["default"])
        self.move_servo(8, self.servos[8]["default"])
        self.move_servo(9, self.servos[9]["default"])
        self.move_servo(10, self.servos[10]["default"])
        self.move_servo(11, self.servos[11]["default"])
        self.move_servo(12, self.servos[12]["default"])
        
        GPIO.output(self.prikthai["front"]["in_a"], GPIO.HIGH)
        GPIO.output(self.prikthai["front"]["in_b"], GPIO.HIGH)
        self.pwm_f.ChangeDutyCycle(100)
        
        GPIO.output(self.prikthai["rear"]["in_a"], GPIO.HIGH)
        GPIO.output(self.prikthai["rear"]["in_b"], GPIO.HIGH)
        self.pwm_r.ChangeDutyCycle(100)
    
    def stop(self):
        GPIO.output(self.prikthai["front"]["in_a"], GPIO.LOW)
        GPIO.output(self.prikthai["front"]["in_b"], GPIO.LOW)
        self.pwm_f.ChangeDutyCycle(0)
        
        GPIO.output(self.prikthai["rear"]["in_a"], GPIO.LOW)
        GPIO.output(self.prikthai["rear"]["in_b"], GPIO.LOW)
        self.pwm_r.ChangeDutyCycle(0)
            

    def move_servo(self, servo_id, position):
        # Move the servo to the specified position if it's within the min and max range
        servo = self.servos[servo_id]
        
        GPIO.output(self.prikthai["front"]["in_a"], GPIO.LOW if servo_id ==1 else GPIO.HIGH)
        GPIO.output(self.prikthai["front"]["in_b"], GPIO.LOW if servo_id ==4 else GPIO.HIGH)
        GPIO.output(self.prikthai["rear"]["in_a"], GPIO.LOW if servo_id ==7 else GPIO.HIGH)
        GPIO.output(self.prikthai["rear"]["in_b"], GPIO.LOW if servo_id ==10 else GPIO.HIGH)
        
        if servo["min_angle"] <= position <= servo["max_angle"]:
            servo["servo"].move(position)
            print(f"Servo {servo_id} moved to {position}")
        else:
            print(f"Position {position} is out of range for servo {servo_id}")

    def move_forward(self):
        # Move forward pattern (left front and right back, then right front and left back)
        print("Moving forward")
        # Move left front and right back forward
        self.move_servo(1, self.servos[1]["high"])  # Front Left up
        self.move_servo(10, self.servos[10]["high"])# Back Right up
        time.sleep(0.5)
        self.move_servo(1, self.servos[1]["low"])   # Front Left down
        self.move_servo(10, self.servos[10]["low"]) # Back Right down
        time.sleep(1)
        
        # Move right front and left back forward
        self.move_servo(4, self.servos[4]["high"])  # Front Right up
        self.move_servo(7, self.servos[7]["high"])  # Back Left up
        time.sleep(0.5)
        self.move_servo(4, self.servos[4]["low"])   # Front Right down
        self.move_servo(7, self.servos[7]["low"])   # Back Left down
        time.sleep(1)

    def move_backward(self):
        # Move backward pattern (reverse of forward)
        print("Moving backward")
        # Move right front and left back backward
        self.move_servo(4, self.servos[4]["low"])   # Front Right up
        self.move_servo(7, self.servos[7]["low"])   # Back Left up
        time.sleep(0.5)
        self.move_servo(4, self.servos[4]["high"])  # Front Right down
        self.move_servo(7, self.servos[7]["high"])  # Back Left down
        time.sleep(1)
        
        # Move left front and right back backward
        self.move_servo(1, self.servos[1]["low"])   # Front Left up
        self.move_servo(10, self.servos[10]["low"]) # Back Right up
        time.sleep(0.5)
        self.move_servo(1, self.servos[1]["high"])  # Front Left down
        self.move_servo(10, self.servos[10]["high"])# Back Right down
        time.sleep(1)

    def turn_left(self):
        # Turn left by moving right side servos forward and left side backward
        print("Turning left")
        self.move_servo(1, self.servos[1]["low"])   # Front Left forward
        self.move_servo(7, self.servos[7]["low"])   # Back Left forward
        self.move_servo(4, self.servos[4]["high"])  # Front Right backward
        self.move_servo(10, self.servos[10]["high"])# Back Right backward
        time.sleep(1)

    def turn_right(self):
        # Turn right by moving left side servos forward and right side backward
        print("Turning right")
        self.move_servo(1, self.servos[1]["high"])  # Front Left backward
        self.move_servo(7, self.servos[7]["high"])  # Back Left backward
        self.move_servo(4, self.servos[4]["low"])   # Front Right forward
        self.move_servo(10, self.servos[10]["low"]) # Back Right forward
        time.sleep(1)

    def reset(self):
        # Reset all servos to their default positions
        print("Resetting servos to default")
        for servo_id, servo_data in self.servos.items():
            self.move_servo(servo_id, servo_data["default"])
            time.sleep(0.5)

# Usage example:
if __name__ == "__main__":
    robot = Robot("/dev/ttyUSB0")
    robot.move_forward()
    time.sleep(2)
    robot.move_backward()
    time.sleep(2)
    robot.turn_left()
    time.sleep(2)
    robot.turn_right()
    time.sleep(2)
    robot.reset()
