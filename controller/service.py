from flask import Flask, request, jsonify, render_template, Response
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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

@app.route('/send_command', methods=['POST'])
def send_command():
    command = request.json.get('command')
    if command in ['a', 'w', 's', 'd']:
        ros2_node.publish_command(command)
        return jsonify({'status': 'success', 'message': f'Command {command} sent to ROS 2'})
    else:
        return jsonify({'status': 'error', 'message': 'Invalid command'}), 400
    
def generate_frames():
    # Open the video capture device (change 0 to the index of your USB capture card if needed)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError("Could not start video capture.")

    while True:
        # Capture frame-by-frame
        success, frame = cap.read()

        if not success:
            break

        # Encode the frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # Concatenate frame bytes into a response for MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()

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