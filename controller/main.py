from flask import Flask, request, jsonify, render_template, Response
import zmq
import threading
import cv2

app = Flask(__name__)

# Set up ZeroMQ REQ socket to communicate with ROS 2
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.137.1:5555")  # Connect to the ROS 2 node

@app.route('/')
def index():
    """Route for rendering the index page."""
    return render_template('index.html')

@app.route('/send_command', methods=['POST'])
def send_command():
    """Route for sending commands to the ROS 2 node."""
    command = request.json.get('command')
    if command in ['a', 'w', 's', 'd', 'status']:
        # Send the command to ROS 2 via ZeroMQ
        socket.send_string(command)

        # Receive the response from ROS 2 node
        response = socket.recv_string()
        return jsonify({'status': 'success', 'message': response})
    else:
        return jsonify({'status': 'error', 'message': 'Invalid command'}), 400

def generate_frames():
    """Generator function for streaming video frames from the camera."""
    cap = cv2.VideoCapture(0)  # Ensure the correct device index is used

    if not cap.isOpened():
        raise RuntimeError("Could not start video capture. Check the camera connection or permissions.")

    while True:
        success, frame = cap.read()
        if not success:
            print("Failed to grab a frame from the camera.")
            break

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            print("Failed to encode the frame.")
            continue

        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()
    print("Video capture stopped.")

@app.route('/video_feed')
def video_feed():
    """Route for video streaming."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Run Flask server
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        # Ensure all OpenCV resources are cleaned up
        cv2.destroyAllWindows()
        print("Application terminated gracefully.")
