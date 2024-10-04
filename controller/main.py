from flask import Flask, request, jsonify, render_template, Response
import zmq
import os
import sys
import cv2
import socket
import concurrent.futures
import threading
import time

app = Flask(__name__)

NETWORK = '192.168.137.'
START_IP = 2
END_IP = 254
REQUIRED_PORTS = [5001, 5002]  # Both ports must be open

# Function to check if a specific port is open on a given IP
def is_port_open(ip, port):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(1)  # Set a timeout of 1 second
            result = sock.connect_ex((ip, port))  # Try to connect to the IP and port
            return result == 0  # Returns True if the port is open
    except Exception as e:
        print(f"Error scanning {ip}:{port} - {e}")
        return False

# Function to scan a single IP address for the required ports
def scan_ip(ip):
    # Check if both required ports are open
    ports_open = [is_port_open(ip, port) for port in REQUIRED_PORTS]
    if all(ports_open):
        print(f"Found IP with both ports open: {ip}")
        return ip
    return None

# Main function to scan the network range
def scan_network():
    found_ip = None

    # Use ThreadPoolExecutor to scan multiple IPs concurrently for efficiency
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
        futures = {executor.submit(scan_ip, f"{NETWORK}{i}"): i for i in range(START_IP, END_IP + 1)}
        for future in concurrent.futures.as_completed(futures):
            ip = future.result()
            if ip:
                found_ip = ip
                break  # Stop once an IP with both required ports is found

    if found_ip:
        print(f"Connecting to {found_ip} with required ports open.")
        return found_ip
    else:
        print("No suitable IP with the required ports was found.")
        return None

# Initialize ZeroMQ context and sockets
SERVER_IP = None
while SERVER_IP == None:
    SERVER_IP = scan_network()
    if SERVER_IP != None:
        break
    print("Wait 5 second re-scan to host...")
    time.sleep(5)

context = zmq.Context()

# REQ socket for sending commands
socket = context.socket(zmq.REQ)
socket.connect(f"tcp://{SERVER_IP}:5001")
socket.setsockopt(zmq.RCVTIMEO, 2000)

# SUB socket for receiving logs
logs_socket = context.socket(zmq.SUB)
logs_socket.connect(f"tcp://{SERVER_IP}:5002")
logs_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

# Global variable to store received logs
received_logs = []

def receive_logs():
    """Function to continuously receive logs from the ROS 2 node."""
    while True:
        try:
            log_message = logs_socket.recv_string()
            print(f"Received log: {log_message}")
            received_logs.append(log_message)
            if len(received_logs) > 30:
                received_logs.pop(0)
        except zmq.Again:
            # Continue listening if no message was received within the timeout
            continue
        except zmq.ZMQError as e:
            print(f"ZeroMQ error while receiving logs: {e}")
            break

# Start a background thread to receive logs continuously
log_thread = threading.Thread(target=receive_logs, daemon=True)
log_thread.start()

@app.route('/')
def index():
    """Route for rendering the index page."""
    return render_template('index.html', logs=received_logs)

@app.route('/logs', methods=['GET'])
def get_logs():
    """Route for retrieving the collected logs."""
    # Return the received logs as a JSON response
    return jsonify({'logs': received_logs})

@app.route('/send_command', methods=['POST'])
def send_command():
    """Route for sending commands to the ROS 2 node."""
    command = request.json.get('command')
    if command in ['start', 'stop', 'forward', 'backward', 'left', 'right', 'status', 'health_check']:
        try:
            # Send the command to ROS 2 via ZeroMQ
            socket.send_string(command)
            
            # Receive the response from ROS 2 node with a timeout
            response = socket.recv_string()
            return jsonify({'status': 'success', 'message': response})
        except zmq.error.Again:
            # This error is raised when the receive operation times out
            return jsonify({'status': 'error', 'message': 'Request timed out'}), 500
        except zmq.ZMQError as e:
            # Handle specific ZeroMQ error
            return jsonify({'status': 'error', 'message': str(e)}), 500
        except Exception as e:
            # Handle general errors and restart the Flask server
            print(f"An unexpected error occurred: {str(e)}")
            return jsonify({'status': 'error', 'message': 'An unexpected error occurred'}), 500
    else:
        return jsonify({'status': 'error', 'message': 'Invalid command'}), 400

@app.route('/video_feed')
def video_feed():
    """Route for video streaming."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames():
    """Generator function for streaming video frames from the camera."""
    cap = cv2.VideoCapture(0)  # Ensure the correct device index is used

    if not cap.isOpened():
        print("Could not start video capture. Check the camera connection or permissions.")
        return fallback_image()  # Show a 'No Signal' image

    while True:
        success, frame = cap.read()
        if not success:
            print("Failed to grab a frame from the camera.")
            return fallback_image()  # Show a 'No Signal' image

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            print("Failed to encode the frame.")
            continue

        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()
    print("Video capture stopped.")

def fallback_image():
    """Generates a fallback image with 'No Signal' text."""
    # Create a blank image (black background)
    no_signal_img = np.zeros((480, 640, 3), dtype=np.uint8)

    # Add 'No Signal' text to the image
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(no_signal_img, "No Signal", (150, 240), font, 2, (255, 255, 255), 2, cv2.LINE_AA)

    # Encode the fallback image as JPEG
    ret, buffer = cv2.imencode('.jpg', no_signal_img)
    if not ret:
        raise RuntimeError("Failed to encode the fallback image.")

    frame = buffer.tobytes()
    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

if __name__ == '__main__':
    try:
        # Run Flask server
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=True)
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        # Ensure all OpenCV resources are cleaned up
        cv2.destroyAllWindows()
        print("Application terminated gracefully.")
