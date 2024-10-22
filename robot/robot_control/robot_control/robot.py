import RPi.GPIO as GPIO
import time
import threading
import utils
class Robot:
    def __init__(self):
        self.servos = utils.servos
        for servo_data in self.servos.values():
            servo = servo_data["servo"]
            servo.set_angle_limits(servo_data["min_angle"], servo_data["max_angle"])

        self.prikthai = utils.delays
        
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
            thread = threading.Thread(target=utils.move_servo, args=(servo_id, self.servos[servo_id]["default"]))
            threads.append(thread)

        # Start all threads
        for thread in threads:
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

    def start(self):
        self.position_default()
        
        self.set_delay_up(1,1)
        self.set_delay_up(1,2)
        self.pwm_f.ChangeDutyCycle(100)
        
        self.set_delay_up(2,1)
        self.set_delay_up(2,2)
        self.pwm_r.ChangeDutyCycle(100)
    
    def stop(self):
        self.set_delay_down(1,1)
        self.set_delay_down(1,2)
        self.pwm_f.ChangeDutyCycle(0)
        
        self.set_delay_down(2,1)
        self.set_delay_down(2,2)
        self.pwm_r.ChangeDutyCycle(0) 

    def set_delay_up(self, position, ab):
        """
        position = 1 is front
        position = 2 is rear
        ab = 1 is in_a (left)
        ab = 2 is in_b (right)
        """
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
        """
        position = 1 is front
        position = 2 is rear
        ab = 1 is in_a (left)
        ab = 2 is in_b (right)
        """
        if position == 1:
            position = "front"
        else:
            position = "rear"
        if ab == 1:
            ab = "in_a"
        else:
            ab = "in_b"
        GPIO.output(self.prikthai[position][ab], GPIO.LOW) 
    
    def move_forward(self):
        servo_id = 3
        # utils.move_servo(servo_id, utils.get_servo_default(servo_id))
        utils.move_servo(servo_id, utils.get_servo_default(servo_id)-15)
        # utils.move_servo(servo_id, utils.get_servo_default(servo_id)+15)
        # utils.smooth_move(servo=utils.get_servo(servo_id), start_angle=(utils.get_servo_default(servo_id) - 15), target_angle=(utils.get_servo_default(servo_id)), delay=0.025)
        # utils.smooth_move(servo=utils.get_servo(servo_id), start_angle=(utils.get_servo_default(servo_id)), target_angle=(utils.get_servo_default(servo_id) + 15), delay=0.025)
        utils.move_servo(servo_id, utils.get_servo_default(servo_id))
        
    def move_backward(self):
        print("move backward")

    def turn_left(self):
        print("Turning left")

    def turn_right(self):
        # Turn right by moving left side servos forward and right side backward
        print("Turning right")
        time.sleep(1)

    def reset(self):
        print("Resetting servos to default")
        self.position_default()
        
            
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
        print("h: Swing Servo")
        print("j: Check Servo Position")
        print("f: Reset to Default Position")
        print("q: Quit")

        # Command loop
        while True:
            command = input("Enter command: ").lower()

            if command == "w":
                robot.move_forward()
            elif command == "h":
                servo = input("Enter servo: ")
                low = input("Enter low limit: ")
                high = input("Enter high limit: ")
                if servo.isdigit() and low.isdigit() and high.isdigit():
                    robot.swing_servo(int(servo), int(low), int(high))
                else:
                    print("Invalid input. Please enter numbers.")
            elif command == "j":
                try:
                    servo_id = int(input("Enter the servo ID to check position: "))
                    position = int(input("Enter position: "))
                    robot.move_servo(servo_id, position)
                except ValueError:
                    print("Invalid input. Please enter numbers.")
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
