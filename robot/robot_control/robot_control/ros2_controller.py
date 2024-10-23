import rclpy
from rclpy.node import Node
import zmq
from datetime import datetime
import pytz
from .robot import Robot

class ROS2Controller(Node):
    def __init__(self):
        super().__init__('robot_client')
        self.pub_socket = None
        self.rep_socket = None
        self.zmq_context = None
        self.connected = False
        self.shutdown_called = False  # Flag to track if shutdown has been called
        self.setup_socket()
        self.robot = Robot(logs = self.logs)

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
            # REP port 5001 - for resolving commands
            self.rep_socket = self.zmq_context.socket(zmq.REP)
            self.rep_socket.bind("tcp://*:5001")

            # PUB port 5002 - for sending logs
            self.pub_socket = self.zmq_context.socket(zmq.PUB)
            self.pub_socket.bind("tcp://*:5002")
            self.logs('ZeroMQ is started')
            self.connected = True
        except Exception as e:
            self.connected = False
            self.logs(f"Failed to bind ZeroMQ socket: {e}", 'error')

    def listen_for_commands(self):
        if self.rep_socket and not self.rep_socket.closed:  # Check if rep_socket is valid
            try:
                # Wait for a command from the client
                command = self.rep_socket.recv_string(flags=zmq.NOBLOCK)
                self.logs(f"Received command: {command}")
                self.handle_command(command)
            except zmq.Again:
                pass

    def handle_command(self, command):
        try:
            if command == "start":
                self.robot.start()
                self.send_response('Start ok')
            elif command == "stop":
                self.robot.stop()
                self.send_response('Stop ok')
            elif command == "forward":
                self.robot.move_forward()
                self.send_response('move_forward ok')
            elif command == "backward":
                self.robot.move_backward()
                self.send_response('move_backward ok')
            elif command == "left":
                self.robot.turn_left()
                self.send_response('turn_left ok')
            elif command == "right":
                self.robot.turn_right()
                self.send_response('turn_right ok')
            elif command == "default":
                self.robot.position_default()
                self.send_response('position_default ok')
            else:
                self.send_response('Invalid command. Try again.')
        except Exception as e:
            self.logs(f"Command handling failed: {str(e)}", 'error')


    def send_response(self, response):
        if self.rep_socket and not self.rep_socket.closed:  # Check if rep_socket is valid
            try:
                self.rep_socket.send_string(response)
                self.logs(f"Sent response: {response}")
            except zmq.ZMQError as e:
                self.logs(f"Failed to send response: {e}", 'error')

    def ok(self):
        return self.connected

    def shutdown(self):
        # Set the shutdown flag to avoid repeated shutdowns
        if self.shutdown_called:
            return

        self.shutdown_called = True

        # Avoid using sockets after they are closed
        if self.pub_socket and not self.pub_socket.closed:
            self.pub_socket.close()
        if self.rep_socket and not self.rep_socket.closed:
            self.rep_socket.close()
        if self.zmq_context:
            self.zmq_context.term()

        # Log shutdown message before fully shutting down the ROS 2 context
        if self.connected:
            self.logs("Cleaned up ZeroMQ resources.")
        self.connected = False

def main(args=None):
    rclpy.init(args=args)
    node = ROS2Controller()
    try:
        while rclpy.ok() and node.ok():
            node.logs("Wait for Command...")  # Periodically send logs
            node.listen_for_commands()  # Listen for incoming commands
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        node.logs('Robot stopped','warn')
    except Exception as e:
        node.logs(f"An error occurred: {str(e)}", 'error')
    finally:
        node.shutdown()
        if not node.shutdown_called:
            rclpy.shutdown()  # Ensure shutdown is called only once

if __name__ == '__main__':
    main()
