import RPi.GPIO as GPIO
import time
import threading
# from . import utils
import utils
class Robot:
    def __init__(self, logs=None):
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
        self.logs = logs
        self.send_logs('echo from robot to ros')
    def send_logs(self,message = ""):
        if(self.logs != None):
            self.logs(message)
                
    def position_default(self, servos=range(1,13)):
        threads = []
        self.send_logs('set default position')
        # Create a thread for each servo movement
        for servo_id in servos:
            thread = threading.Thread(target=utils.move_servo, args=(servo_id, self.servos[servo_id]["default"]))
            self.send_logs(f'move servo {servo_id} to default position ({self.servos[servo_id]["default"]})')
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
        self.send_logs('set power to 100%')
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
        self.send_logs('set power to 0%')
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
        self.send_logs(f'set delay up fr: {position} ab: {ab} delay: {delay}')
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
        self.send_logs(f'set delay down fr: {position} ab: {ab} delay: {delay}')
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
    
    def move_top_left_up(self):
        fr = 1
        ab = 1
        self.delay_down(fr,ab, 5)
        self.send_logs("top left up")
        utils.top_left_up()
        self.delay_up(fr,ab)
    
    def move_top_right_up(self):
        fr = 1
        ab = 2
        self.delay_down(fr,ab, 5)
        self.send_logs("top right up")
        utils.top_right_up()
        self.delay_up(fr, ab)
    
    def move_bot_left_up(self):
        fr = 2
        ab = 1
        self.delay_down(fr,ab, 5)
        self.send_logs("bot left up")
        utils.bot_left_up()
        self.delay_up(fr,ab)
            
    def move_bot_right_up(self):
        fr = 2
        ab = 2
        self.delay_down(fr,ab, 5)
        self.send_logs("bot right up")
        utils.bot_right_up()
        self.delay_up(fr, ab)
        
    def move_top_left_down(self):
        fr = 1
        ab = 1
        self.delay_down(fr,ab, 5)
        self.send_logs("top left down")
        utils.top_left_down()
        self.delay_up(fr,ab)
    
    def move_top_right_down(self):
        fr = 1
        ab = 2
        self.delay_down(fr,ab, 5)
        self.send_logs("top right down")
        utils.top_right_down()
        self.delay_up(fr, ab)
    
    def move_bot_left_down(self):
        fr = 2
        ab = 1
        self.delay_down(fr,ab, 5)
        self.send_logs("bot left down")
        utils.bot_left_down()
        self.delay_up(fr,ab)
            
    def move_bot_right_down(self):
        fr = 2
        ab = 2
        self.delay_down(fr,ab, 5)
        self.send_logs("bot right down")
        utils.bot_right_down()
        self.delay_up(fr, ab)
                 
    def move_forward(self):
        self.move_top_left_up()
        self.move_top_right_up()
        self.move_bot_left_up()
        self.move_bot_right_up()
        self.position_default([1,4,7,10])       
        
    def move_backward(self):
        self.move_top_left_down()
        self.move_top_right_down()
        self.move_bot_left_down()
        self.move_bot_right_down()
        self.position_default([1,4,7,10])      

    def turn_left(self):
        self.move_top_left_down()
        self.move_top_right_up()
        self.move_bot_left_down()
        self.move_bot_right_up()
        self.position_default([1,4,7,10])  

    def turn_right(self):
        self.move_top_left_up()
        self.move_top_right_down()
        self.move_bot_left_up()
        self.move_bot_right_down()
        self.position_default([1,4,7,10])  

    def reset(self):
        print("Resetting servos to default")
        self.position_default()
        