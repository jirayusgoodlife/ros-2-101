from pylx16a.lx16a import LX16A
import RPi.GPIO as GPIO
import time
import threading

class Robot:
    def __init__(self):
        # Initialize LX16A communication
        LX16A.initialize('/dev/ttyUSB0')
        self.servos = {
            1: {"servo": LX16A(1), "min_angle": 30, "max_angle": 155,  "low": 90, "high": 150, "swing": [55, 120], "default": 90},
            2: {"servo": LX16A(2), "min_angle": 150, "max_angle": 210, "low": 160, "high": 190, "default": 180},
            3: {"servo": LX16A(3), "min_angle": 30, "max_angle": 150, "low": 90, "high": 40, "default": 115},
            4: {"servo": LX16A(4), "min_angle": 80, "max_angle": 200, "low": 150, "high": 80, "swing": [180, 120], "default": 150},
            5: {"servo": LX16A(5), "min_angle": 140, "max_angle": 210, "low": 160, "high": 240, "default": 190},
            6: {"servo": LX16A(6), "min_angle": 40, "max_angle": 150, "low": 100, "high": 50, "default": 145},
            7: {"servo": LX16A(7), "min_angle": 80, "max_angle": 210, "low": 155, "high": 90, "swing": [120, 180], "default": 155},
            8: {"servo": LX16A(8), "min_angle": 31, "max_angle": 120, "low": 110, "high": 60, "default": 50},
            9: {"servo": LX16A(9), "min_angle": 80, "max_angle": 170, "low": 130, "high": 160, "default": 110},
            10: {"servo": LX16A(10), "min_angle": 30, "max_angle": 170, "low": 80, "high": 150, "swing": [80, 120], "default": 80},
            11: {"servo": LX16A(11), "min_angle": 25, "max_angle": 90, "low": 90, "high": 25, "default": 50},
            12: {"servo": LX16A(12), "min_angle": 100, "max_angle": 200, "low": 150, "high": 190, "default": 125},
        }
        
        for servo_data in self.servos.values():
            servo = servo_data["servo"]
            servo.set_angle_limits(servo_data["min_angle"], servo_data["max_angle"])

        
        self.prikthai = { 
            "front": {"in_a": 20, "in_b": 16, "pwm": 12},
            "rear": {"in_a": 26, "in_b": 19, "pwm": 13}
        }
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for direction in self.prikthai.values():
            GPIO.setup(direction["in_a"], GPIO.OUT)
            GPIO.setup(direction["in_b"], GPIO.OUT)
            GPIO.setup(direction["pwm"], GPIO.OUT)
        self.pwm_f = GPIO.PWM(self.prikthai["front"]["pwm"], 5000)
        self.pwm_r = GPIO.PWM(self.prikthai["rear"]["pwm"], 5000) 
        
    def position_default(self):
        threads = []

        # Create a thread for each servo movement
        for servo_id in range(1,13):
            thread = threading.Thread(target=self.move_servo, args=(servo_id, self.servos[servo_id]["default"]))
            threads.append(thread)

        # Start all threads
        for thread in threads:
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()
            
    def smooth_move(self, servo, start_angle, target_angle, step=1, delay=0.01):
        """
        Smoothly move the servo from the start angle to the target angle in small steps.
        """
        if start_angle < target_angle:
            for angle in range(start_angle, target_angle + 1, step):
                servo.move(angle)
                time.sleep(delay)
        else:
            for angle in range(start_angle, target_angle - 1, -step):
                servo.move(angle)
                time.sleep(delay)

    def swing_servo(self, servo_id):
        """
        Swing the servo between low and high swing angles repeatedly.
        """
        servo_info = self.servos[servo_id]
        servo = servo_info["servo"]
        swing_angles = servo_info["swing"]

        while True:
            # Move to the first swing position
            self.smooth_move(servo, swing_angles[0], swing_angles[1])
            # Move back to the original position
            self.smooth_move(servo, swing_angles[1], swing_angles[0])

    def start_servo_swing(self, servo_id):
        """
        Start the swing operation for a servo using a thread.
        """
        thread = threading.Thread(target=self.swing_servo, args=(servo_id,))
        thread.daemon = True  # Ensure the thread stops when the program exits.
        thread.start()
            
    def start(self):
        self.position_default()
        
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

    def set_delay_up(self, position, ab):
        if position == 1:
            position = "front"
        else:
            position = "rear"
        if ab == 1:
            ab = "in_a"
        else:
            ab = "in_b"
            
        GPIO.output(self.prikthai[position][ab], GPIO.HIGH)
      
    def set_delay_down(self, position, ab):
        if position == 1:
            position = "front"
        else:
            position = "rear"
        if ab == 1:
            ab = "in_a"
        else:
            ab = "in_b"
        GPIO.output(self.prikthai[position][ab], GPIO.LOW)

    def move_servo(self, servo_id, position, mode=False):
        servo = self.servos[servo_id]
        
        servo["servo"].move(position)
        
        time.sleep(0.5)
    
    
    def move_forward(self):
    #     print("move_forward")
    #     # head left
        # self.set_delay_down(1,1)
        # time.sleep(3)
        self.move_servo(2, self.servos[2]["high"] + 20)
        self.move_servo(3, self.servos[3]["default"] + 10)
        self.move_servo(1, self.servos[1]["swing"][0])
        self.move_servo(2, self.servos[2]["default"])
        self.move_servo(3, self.servos[3]["default"])
        self.set_delay_up(1,1)
        
        # self.set_delay_down(2,2)
        # time.sleep(3)
    #     # tail right
        self.move_servo(11, self.servos[11]["default"] - 15)
        self.move_servo(10, self.servos[10]["swing"][1])
        self.move_servo(11, self.servos[11]["default"])
        
        self.move_servo(1, self.servos[1]["swing"][1])
        # self.set_delay_up(2,2)
        
        # self.set_delay_down(1,2)
        # time.sleep(3)
        self.move_servo(5, self.servos[5]["default"] + 20)
        self.move_servo(6, self.servos[6]["default"] - 15)
        self.move_servo(4, self.servos[4]["swing"][0])
        self.move_servo(5, self.servos[5]["default"])
        
        self.move_servo(10, self.servos[10]["swing"][0])
        # self.set_delay_up(1,2)
        
        # self.set_delay_down(2,1)
        # time.sleep(3)
        self.move_servo(8, self.servos[8]["default"] - 15)
        self.move_servo(7, self.servos[7]["swing"][0])
        self.move_servo(8, self.servos[8]["default"])
        
        self.move_servo(7, self.servos[7]["swing"][1])
        # self.set_delay_up(2,1)
        

    def move_backward(self):
        print("move_backward")
        self.move_servo(1, self.servos[1]["low"])
        self.move_servo(10, self.servos[10]["low"])
        time.sleep(0.5)
        self.move_servo(1, self.servos[1]["high"])
        self.move_servo(10, self.servos[10]["high"])
        time.sleep(1)
        
        # Move right front and left back forward
        self.move_servo(4, self.servos[4]["low"])
        self.move_servo(7, self.servos[7]["low"])
        time.sleep(0.5)
        self.move_servo(4, self.servos[4]["high"])
        self.move_servo(7, self.servos[7]["high"])

    def turn_left(self):
        print("Turning left")

        print("Turn left completed")

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
        self.position_default()
        
    def swing_servo(self, servo_id, low, high, duration=0.5, cycles=5):
        """Swing the servo between low and high positions for a given number of cycles."""
        servo_id, low, high = int(servo_id), int(low), int(high)  # Convert user inputs to integers
        for _ in range(cycles):
            self.move_servo(servo_id, low)
            time.sleep(duration)
            self.move_servo(servo_id, high)
            time.sleep(duration)    
            
# Usage example:
if __name__ == "__main__":
    robot = Robot()
    try:        
        print("Control your robot:")
        print("w: Move Forward")
        print("s: Move Backward")
        print("a: Turn Left")
        print("d: Turn Right")
        print("t: Start")
        print("y: Stop")
        print("q: Quit")

        while True:
            command = input("Enter command: ").lower()

            if command == "w":
                robot.move_forward()
            elif command == "h":
                servo = input("servo: ")
                low = input("low: ")
                high  = input('high: ')
                if servo.isdigit() and low.isdigit() and high.isdigit():
                    print(f"Swinging servo {servo} between {low} and {high}.")
                    robot.swing_servo(servo, low, high)
            elif command == 'j':
                servo_id = int(input("Enter the servo ID to check position: "))
                position = int(input('position: '))
                robot.move_servo(servo_id, position)                
            elif command == "s":
                robot.move_backward()
            elif command == "a":
                robot.turn_left()
            elif command == "d":
                robot.turn_right()
            elif command == "t":
                robot.start()
            elif command == "y":
                robot.stop()
            elif command == "f":
                robot.position_default()
            elif command == "q":
                print("Exiting...")
                break
            else:
                print("Invalid command. Please try again.")
            
            time.sleep(1)  # Optional: Add delay to prevent rapid consecutive actions

        # Reset the servos before exiting
    except Exception as e:
        print(e)
        robot.reset()
        print("Robot reset and program terminated.")
