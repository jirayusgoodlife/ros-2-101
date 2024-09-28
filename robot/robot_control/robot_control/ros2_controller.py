import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Set up QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Adjust depth as needed
        )

        # Subscription to the command topic with custom QoS settings
        self.subscription = self.create_subscription(
            String,
            'command_topic',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning
        
        # Create a service called 'check_status' that uses the Trigger service type
        self.status_service = self.create_service(
            Trigger, 
            'check_status', 
            self.check_status_callback
        )

        self.get_logger().info('RobotController node has been started with Fast DDS.')

    def listener_callback(self, msg):
        command = msg.data
        # Define the actions based on the received command
        if command == 'w':
            self.get_logger().info('Received command: Move forward')
        elif command == 's':
            self.get_logger().info('Received command: Move backward')
        elif command == 'a':
            self.get_logger().info('Received command: Turn left')
        elif command == 'd':
            self.get_logger().info('Received command: Turn right')
        else:
            self.get_logger().info(f'Received unknown command: {command}')
            
    def check_status_callback(self, request, response):
        # Respond to the service call with a success message
        response.success = True
        response.message = "RobotController is running and responsive."
        self.get_logger().info('Status check requested: Node is running.')
        return response

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the RobotController node
    robot_controller = RobotController()

    # Log the middleware currently being used (Fast DDS)
    robot_controller.get_logger().info(f"Using middleware: {rclpy.get_rmw_implementation_identifier()}")

    # Keep the node running
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info('Shutting down RobotController...')

    # Clean up and shut down the node
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
