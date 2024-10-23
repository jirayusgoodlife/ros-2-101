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
            servo.enable_torque()

        self.prikthai = utils.delays
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for direction in self.prikthai.values():
            GPIO.setup(direction["in_a"], GPIO.OUT)
            GPIO.setup(direction["in_b"], GPIO.OUT)
            GPIO.setup(direction["pwm"], GPIO.OUT)
        self.pwm_f = GPIO.PWM(self.prikthai["front"]["pwm"], 5000)
        self.pwm_r = GPIO.PWM(self.prikthai["rear"]["pwm"], 5000) 
        
    def position_default(self, servos=range(1,13)):
        threads = []

        # Create a thread for each servo movement
        for servo_id in servos:
            thread = threading.Thread(target=utils.move_servo, args=(servo_id, self.servos[servo_id]["default"]))
            threads.append(thread)

        # Start all threads
        for thread in threads:
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

    def start(self):
        # set default position
        self.position_default()
        # set power to 100%
        self.pwm_f.ChangeDutyCycle(100)
        self.pwm_r.ChangeDutyCycle(100)
        
        threads = []
        for fr in range(1,3):
            for ab in range(1,3):
                thread = threading.Thread(target=self.delay_up, args=(fr, ab))
                threads.append(thread)
                
        # Start all threads
        for thread in threads:
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()
    
    def stop(self):
        # set power to 0%
        self.pwm_f.ChangeDutyCycle(0)
        self.pwm_r.ChangeDutyCycle(0) 
        threads = []
        for fr in range(1,3):
            for ab in range(1,3):
                thread = threading.Thread(target=self.delay_down, args=(fr, ab))
                threads.append(thread)
                
        # Start all threads
        for thread in threads:
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

    def delay_up(self, position, ab, delay=1):
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
        time.sleep(delay)
      
    def delay_down(self, position, ab, delay=1):
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
        time.sleep(delay)
    
    def move_forward(self):
        
        fr = 1
        ab = 1
        # self.delay_down(fr,ab, 5)
        utils.top_left_up()
        # self.delay_up(fr,ab)
        
        # fr = 1
        # ab = 2
        # self.delay_down(fr,ab, 5)
        utils.top_right_up()
        # self.delay_up(fr, ab)
        
        # fr = 2
        # ab = 2
        # self.delay_down(fr,ab, 5)
        utils.bot_right_up()
        # self.delay_up(fr, ab)
        
        # fr = 2
        # ab = 1
        # self.delay_down(fr,ab, 5)
        utils.bot_left_up()
        # self.delay_up(fr,ab)
        
        self.position_default([1,4,7,10])       
        
    def move_backward(self):
        
        utils.top_left_down()
        
        utils.top_right_down()
        
        utils.bot_left_down()
        
        utils.bot_right_down()
        
        
        self.position_default([1,4,7,10])      

    def turn_left(self):
        utils.top_left_down()
        utils.top_right_up()
        utils.bot_left_down()
        utils.bot_right_up()
        self.position_default([1,4,7,10])  

    def turn_right(self):
        utils.top_left_up()
        utils.top_right_down()
        utils.bot_left_up()
        utils.bot_right_down()
        self.position_default([1,4,7,10])  

    def reset(self):
        print("Resetting servos to default")
        self.position_default()
        