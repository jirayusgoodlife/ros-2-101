import RPi.GPIO as GPIO
import time
from pylx16a.lx16a import *

# Use BCM numbering
# GPIO.setmode(GPIO.BCM)

# Pin configurations for forward ("f") and reverse ("r")
# prikthai = { 
#     "f": {"in_a": 20, "in_b": 16, "pwm": 12},
#     "r": {"in_a": 26, "in_b": 19, "pwm": 13}
# }

# # Set up the GPIO pins for each motor direction
# for direction in prikthai.values():
#     GPIO.setup(direction["in_a"], GPIO.OUT)
#     GPIO.setup(direction["in_b"], GPIO.OUT)
#     GPIO.setup(direction["pwm"], GPIO.OUT)

# # Set up PWM on both PWM pins (frequency = 5000 Hz)
# pwm_f = GPIO.PWM(prikthai["f"]["pwm"], 5000)  # Forward motor PWM
# pwm_r = GPIO.PWM(prikthai["r"]["pwm"], 5000)  # Reverse motor PWM

# # Start PWM with 0% duty cycle
# pwm_f.start(0)
# pwm_r.start(0)

try:
    LX16A.initialize("/dev/ttyUSB0")
    servos = {
        1: {"servo": LX16A(1), "min_angle": 30.0, "max_angle": 156.0},
        2: {"servo": LX16A(2), "min_angle": 144.0, "max_angle": 210.0},
        3: {"servo": LX16A(3), "min_angle": 36.0, "max_angle": 84.0},
        4: {"servo": LX16A(4), "min_angle": 30.2, "max_angle": 207.6},
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
    for servo_data in servos.values():
        servo = servo_data["servo"]
        servo.set_angle_limits(servo_data["min_angle"], servo_data["max_angle"])
    
    
    while True:
        position = input("servo ")
        servos[1]['servo'].move(position)
        # GPIO.output(prikthai["f"]["in_a"], GPIO.HIGH)
        # GPIO.output(prikthai["f"]["in_b"], GPIO.HIGH)
        # pwm_f.ChangeDutyCycle(100)

        # GPIO.output(prikthai["r"]["in_a"], GPIO.HIGH)
        # GPIO.output(prikthai["r"]["in_b"], GPIO.HIGH)
        # pwm_r.ChangeDutyCycle(100)

        # time.sleep(2)  # Keep motors in brake mode for 2 seconds
        # GPIO.output(prikthai["f"]["in_a"], GPIO.LOW)
        # GPIO.output(prikthai["f"]["in_b"], GPIO.LOW)
        # pwm_f.ChangeDutyCycle(0)

        # GPIO.output(prikthai["r"]["in_a"], GPIO.LOW)
        # GPIO.output(prikthai["r"]["in_b"], GPIO.LOW)
        # pwm_r.ChangeDutyCycle(0)
        # time.sleep(2)  # Keep motors in brake mode for 2 seconds

except KeyboardInterrupt:
    pass

# Stop the PWM signals and clean up the GPIO pins
# pwm_f.stop()
# pwm_r.stop()
# GPIO.cleanup()
