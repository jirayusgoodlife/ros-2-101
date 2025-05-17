import rclpy
from rclpy.node import Node
import zmq
from datetime import datetime
import pytz
from .robot import Robot
import time

import threading
import cv2
import base64  # still imported if needed elsewhere

class ROS2Controller(Node):
    def __init__(self):
        super().__init__('robot_client')
        self.pub_socket = None
        self.rep_socket = None
        self.zmq_context = None
        self.camera_socket = None
        self.connected = False
        self.shutdown_called = False  # Flag to track if shutdown has been called
        self.last_log_message = None
        self.log_repeat_count = 0
        
        self.setup_socket()
        self.robot = Robot(logs=self.logs)
        self.start_camera_stream() # Start the camera stream upon initialization
        
        
        
    def logs(self, message, type_log='info'):
        if not self.shutdown_called:
            # Handle message repeat logic
            if message == self.last_log_message:
                self.log_repeat_count += 1

                # Show only if it’s the 1st, 2nd, or every 10th repeat
                if self.log_repeat_count == 1 or self.log_repeat_count == 2 or self.log_repeat_count % 10 == 0:
                    self._log_and_publish(message, type_log)
            else:
                # New message, reset counter and log
                self.last_log_message = message
                self.log_repeat_count = 1
                self._log_and_publish(message, type_log)

    def _log_and_publish(self, message, type_log):
        if self.pub_socket and not self.pub_socket.closed:
            ros_time = self.get_clock().now().to_msg()
            timestamp = datetime.fromtimestamp(ros_time.sec + ros_time.nanosec / 1e9)

            bangkok_tz = pytz.timezone('Asia/Bangkok')
            timestamp_bangkok = timestamp.astimezone(bangkok_tz)
            formatted_time = timestamp_bangkok.strftime('%Y-%m-%d %H:%M:%S')
            formatted_message = f"[{type_log.upper()}] [{formatted_time}] [{self.get_name()}]: {message}"
            
            try:
                self.pub_socket.send_string(formatted_message)
            except zmq.ZMQError as e:
                self.get_logger().error(f"Failed to send log via ZeroMQ: {e}")

        if type_log == 'info':
            self.get_logger().info(f"{message}")
        elif type_log == 'error':
            self.get_logger().error(f"{message}")
        else:
            self.get_logger().warn(f"{message}")

    def setup_socket(self):
        try:
            self.zmq_context = zmq.Context()

            # REP socket (renamed to SUB to match Flask's PULL)
            self.rep_socket = self.zmq_context.socket(zmq.SUB)
            self.rep_socket.bind("tcp://*:5001")
            self.rep_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            # PUB socket for logs
            self.pub_socket = self.zmq_context.socket(zmq.PUB)
            self.pub_socket.bind("tcp://*:5002")

            # PUB socket for camera feed
            self.camera_socket = self.zmq_context.socket(zmq.PUB)
            self.camera_socket.bind("tcp://*:5003")

            self.logs('ZeroMQ sockets initialized successfully.')
            self.connected = True
        except zmq.ZMQError as e:
            self.connected = False
            self.logs(f"Failed to bind ZeroMQ socket: {e}", 'error')
            raise  # Ensure error propagation

    def listen_for_commands(self):
        if not self.rep_socket or self.rep_socket.closed:
            self.logs("SUB socket is not available.", 'error')
            return

        try:
            # Receive commands from the publisher
            command = self.rep_socket.recv_string(flags=zmq.NOBLOCK)
            self.logs(f"Received command: {command}")
            self.handle_command(command)
        except zmq.Again:  # No message available
            time.sleep(0.1)  # Avoid busy-waiting
        except zmq.ZMQError as e:
            self.logs(f"ZeroMQ Error: {e}", 'error')

    def handle_command(self, command):
        if not self.robot:
            self.logs("Robot instance not initialized.", 'error')
            return

        try:
            if command == "start":
                self.robot.start()
                self.logs('Start ok')
            elif command == "stop":
                self.robot.stop()
                self.logs('Stop ok')
            elif command == "forward":
                self.robot.move_forward()
                self.logs('move_forward ok')
            elif command == "backward":
                self.robot.move_backward()
                self.logs('move_backward ok')
            elif command == "left":
                self.robot.turn_left()
                self.logs('turn_left ok')
            elif command == "right":
                self.robot.turn_right()
                self.logs('turn_right ok')
            elif command == "default":
                self.robot.position_default()
                self.logs('position_default ok')
            elif command == "mode_delay_on":
                self.robot.turn_off_delay = False
                self.logs('turn_on_delay ok')
            elif command == "mode_delay_off":
                self.robot.turn_off_delay = True
                
                self.logs('turn_off_delay 1 1 ok')
                self.robot.delay_down(1,1, 5)
                
                self.logs('turn_off_delay 1 2 ok')
                self.robot.delay_down(1,2, 5)
                
                self.logs('turn_off_delay 2 1 ok')
                self.robot.delay_down(2,1, 5)
                
                self.logs('turn_off_delay 2 2 ok')
                self.robot.delay_down(2,2, 5)
                
                self.logs('turn_off_delay ok')
            else:
                self.logs('Invalid command. Try again.', 'warn')
        except Exception as e:
            self.logs(f"Command handling failed: {e}", 'error')
            
    def start_camera_stream(self):
        self.logs("start_camera_stream started")
        def stream():
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                self.logs("Failed to open webcam", "error")
                return

            self.logs("Camera stream started on port 5003 (PUB)")

            while self.ok():
                ret, frame = cap.read()
                if not ret:
                    continue

                ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if not ret:
                    continue

                try:
                    self.camera_socket.send(buffer.tobytes())
                    # DEBUG Logs camera
                    # self.logs(f"Sent {len(buffer.tobytes())} bytes of camera frame", "info") 
                except zmq.ZMQError as e:
                    self.logs(f"Failed to send camera frame: {e}", "error")

                time.sleep(0.01)

            cap.release()
            self.logs("Camera stream stopped")

        threading.Thread(target=stream, daemon=True).start()
        
    def shutdown(self):
        if self.shutdown_called:
            self.logs("Shutdown already initiated.", 'warn')
            return

        self.shutdown_called = True
        self.logs("Shutting down...")

        # Close ZeroMQ sockets safely
        if self.pub_socket and not self.pub_socket.closed:
            self.pub_socket.close()
        if self.rep_socket and not self.rep_socket.closed:
            self.rep_socket.close()
        if self.zmq_context:
            self.zmq_context.term()

        self.connected = False

    def ok(self):
        return rclpy.ok() and self.connected
     
def main(args=None):
    rclpy.init(args=args)
    node = ROS2Controller()
    print("ROS 2 Controller Node started. Camera stream initialized.")
    node.logs("ROS 2 Controller Node started. Camera stream initialized.")

    try:
        while rclpy.ok() and node.ok():
            node.logs("Waiting for command...")  # Periodic log
            node.listen_for_commands()
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        node.logs('Robot stopped.', 'warn')
    except Exception as e:
        node.logs(f"An error occurred: {e}", 'error')
    finally:
        node.shutdown()
        if not node.shutdown_called:
            rclpy.shutdown()

if __name__ == '__main__':
    main()