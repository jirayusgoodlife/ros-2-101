import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger 

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            String,
            'command_topic',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Create a service called 'check_status' that uses the Trigger service type
        self.status_service = self.create_service(
            Trigger, 
            'check_status', 
            self.check_status_callback
        )

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
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
