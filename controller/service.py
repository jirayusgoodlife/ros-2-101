from flask import Flask, request, jsonify, render_template, Response
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import threading
import cv2

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
    
# Route for rendering the index page
@app.route('/')
def index():
    return render_template('index.html')

def check_ros2_status():
    """Checks the status of the ROS 2 node using a service call."""
    client = ros2_node.create_client(Trigger, 'check_status')
    if not client.wait_for_service(timeout_sec=2.0):
        return False, "ROS 2 service 'check_status' not available."

    request = Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(ros2_node, future)
    if future.result() is not None:
        return future.result().success, future.result().message
    else:
        return False, "Failed to communicate with ROS 2 service."
    
@app.route('/send_command', methods=['POST'])
def send_command():
    # Check if ROS 2 node is responsive before sending the command
    status, message = check_ros2_status()
    if not status:
        return jsonify({'status': 'error', 'message': f'ROS 2 connection error: {message}'}), 500

    command = request.json.get('command')
    if command in ['a', 'w', 's', 'd']:
        ros2_node.publish_command(command)
        return jsonify({'status': 'success', 'message': f'Command {command} sent to ROS 2'})
    else:
        return jsonify({'status': 'error', 'message': 'Invalid command'}), 400
    
def generate_frames():
    # Open the video capture device (0 is usually the default camera)
    cap = cv2.VideoCapture(0)  # Ensure the correct device index is used

    if not cap.isOpened():
        raise RuntimeError("Could not start video capture. Check the camera connection or permissions.")

    while True:
        # Capture frame-by-frame
        success, frame = cap.read()

        if not success:
            print("Failed to grab a frame from the camera.")
            break

        # Encode the frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            print("Failed to encode the frame.")
            continue

        frame = buffer.tobytes()

        # Concatenate frame bytes into a response for MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()
    print("Video capture stopped.")

@app.route('/video_feed')
def video_feed():
    # Video streaming route
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# Main entry point
if __name__ == '__main__':
    try:
        ros2_threading = threading.Thread(target=ros2_thread)
        ros2_threading.start()
        app.run(host='0.0.0.0', port=5000)
    finally:
        cv2.destroyAllWindows()