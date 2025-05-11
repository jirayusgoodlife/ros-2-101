import rclpy
from rclpy.node import Node
import zmq
from datetime import datetime
import pytz
from .robot import Robot
import time

class ROS2Controller(Node):
    def __init__(self):
        super().__init__('robot_client')
        self.pub_socket = None
        self.rep_socket = None
        self.zmq_context = None
        self.connected = False
        self.shutdown_called = False  # Flag to track if shutdown has been called
        self.setup_socket()
        self.robot = Robot(logs=self.logs)

    def logs(self, message, type_log='info'):
        # Only log if ROS 2 context is still active
        if not self.shutdown_called:
            if self.pub_socket and not self.pub_socket.closed:  # Check if pub_socket is valid
                
                ros_time = self.get_clock().now().to_msg()  # Get the current time from ROS 2
                timestamp = datetime.fromtimestamp(ros_time.sec + ros_time.nanosec / 1e9)  # Convert to naive datetime

                # Set timezone to Asia/Bangkok (UTC+7)
                bangkok_tz = pytz.timezone('Asia/Bangkok')
                timestamp_bangkok = timestamp.astimezone(bangkok_tz)  # Convert to Bangkok timezone

                # Format the timestamp to the desired format: yyyy-mm-dd hh:mm:ss
                formatted_time = timestamp_bangkok.strftime('%Y-%m-%d %H:%M:%S')
                formatted_message = f"[{type_log.upper()}] [{formatted_time}] [{self.get_name()}]: {message}"
                
                try:
                    self.pub_socket.send_string(formatted_message)
                except zmq.ZMQError as e:
                    self.get_logger().error(f"Failed to send log via ZeroMQ: {e}")

            # ROS 2 logging
            if type_log == 'info':
                self.get_logger().info(f"{message}")
            elif type_log == 'error':
                self.get_logger().error(f"{message}")
            else:
                self.get_logger().warn(f"{message}")

    def setup_socket(self):
        try:
            self.zmq_context = zmq.Context()

            # REP socket
            self.rep_socket = self.zmq_context.socket(zmq.SUB)
            self.rep_socket.bind("tcp://*:5001")
            self.rep_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            # PUB socket
            self.pub_socket = self.zmq_context.socket(zmq.PUB)
            self.pub_socket.bind("tcp://*:5002")

            self.logs('ZeroMQ sockets initialized successfully.')
            self.connected = True
        except zmq.ZMQError as e:
            self.connected = False
            self.logs(f"Failed to bind ZeroMQ socket: {e}", 'error')
            raise  # Ensure error propagation to catch startup issues.

    
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
                self.logs('turn_off_delay ok')
            else:
                self.logs('Invalid command. Try again.', 'warn')
        except Exception as e:
            self.logs(f"Command handling failed: {e}", 'error')

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
    
def start_camera_stream(self):
        def stream():
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                self.logs("Failed to open webcam", "error")
                return
            
            self.logs("Camera stream started")

            while self.ok():
                ret, frame = cap.read()
                if not ret:
                    continue

                # Encode to JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    continue

                # Optional: Base64 encode for safer transport (or just send raw bytes)
                jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                self.pub_socket.send_string(f"video:{jpg_as_text}")

                time.sleep(0.05)  # ~20 FPS

            cap.release()

        threading.Thread(target=stream, daemon=True).start()
        
def main(args=None):
    rclpy.init(args=args)
    node = ROS2Controller()
    try:
        node.start_camera_stream()
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
            rclpy.shutdown()  # Ensure proper shutdown

if __name__ == '__main__':
    main()
