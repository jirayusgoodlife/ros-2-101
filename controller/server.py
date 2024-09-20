from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

app = Flask(__name__)
ros2_node = None


class FlaskROS2Publisher(Node):
    def __init__(self):
        super().__init__('flask_publisher')
        self.publisher_ = self.create_publisher(String, 'command_topic', 10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: {command}')


def ros2_thread():
    global ros2_node
    rclpy.init()
    ros2_node = FlaskROS2Publisher()
    rclpy.spin(ros2_node)


@app.route('/send_command', methods=['POST'])
def send_command():
    command = request.json.get('command')
    if command in ['a', 'w', 's', 'd']:
        ros2_node.publish_command(command)
        return jsonify({'status': 'success', 'message': f'Command {command} sent to ROS 2'})
    else:
        return jsonify({'status': 'error', 'message': 'Invalid command'}), 400


if __name__ == '__main__':
    ros2_threading = threading.Thread(target=ros2_thread)
    ros2_threading.start()
    app.run(host='0.0.0.0', port=5000)
