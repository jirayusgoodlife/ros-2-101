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

SERVER_IP = None  # Track the current server IP dynamically
context = zmq.Context()

# REQ socket for sending commands
socket = context.socket(zmq.REQ)
socket.setsockopt(zmq.RCVTIMEO, 2000)

# SUB socket for receiving logs
logs_socket = context.socket(zmq.SUB)
logs_socket.setsockopt_string(zmq.SUBSCRIBE, "")

received_logs = []  # Store received logs

# Function to check if a specific port is open on a given IP
def is_port_open(ip, port):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(1)  # Set a timeout of 1 second
            result = sock.connect_ex((ip, port))  # Try to connect
            return result == 0  # True if the port is open
    except Exception as e:
        print(f"Error scanning {ip}:{port} - {e}")
        return False

# Function to scan a single IP address for the required ports
def scan_ip(ip):
    ports_open = [is_port_open(ip, port) for port in REQUIRED_PORTS]
    if all(ports_open):
        print(f"Found IP with both ports open: {ip}")
        return ip
    return None

# Main function to scan the network range
def scan_network():
    found_ip = None
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
        futures = {executor.submit(scan_ip, f"{NETWORK}{i}"): i for i in range(START_IP, END_IP + 1)}
        for future in concurrent.futures.as_completed(futures):
            ip = future.result()
            if ip:
                found_ip = ip
                break
    return found_ip

def connect_to_server(ip):
    """Attempt to connect to the server with the given IP."""
    global SERVER_IP
    try:
        # Attempt to connect using ZeroMQ (update if needed)
        socket.connect(f"tcp://{ip}:5001")
        logs_socket.connect(f"tcp://{ip}:5002")
        SERVER_IP = ip
        return True
    except Exception as e:
        print(f"Failed to connect to {ip}: {e}")
        return False

@app.route('/')
def index():
    """Render the index page."""
    return render_template('index.html', logs=received_logs)

@app.route('/logs', methods=['GET'])
def get_logs():
    """Retrieve the collected logs."""
    return jsonify({'logs': received_logs})

@app.route('/set_ip', methods=['POST'])
def set_ip():
    """Set the server IP manually."""
    ip = request.json.get('ip')
    if connect_to_server(ip):
        return jsonify({'status': 'success', 'message': f'Connected to {ip}'})
    else:
        return jsonify({'status': 'error', 'message': f'Failed to connect to {ip}'}), 500

@app.route('/scan_network', methods=['POST'])
def scan_network_route():
    """Scan the network and connect to the first available server."""
    ip = scan_network()
    if ip and connect_to_server(ip):
        return jsonify({'status': 'success', 'message': f'Connected to {ip}'})
    else:
        return jsonify({'status': 'error', 'message': 'No suitable server found'}), 404

@app.route('/send_command', methods=['POST'])
def send_command():
    """Send commands to the ROS 2 node."""
    command = request.json.get('command')
    if command in ['start', 'stop', 'forward', 'backward', 'left', 'right', 'default']:
        try:
            socket.send_string(command)
            response = socket.recv_string()
            return jsonify({'status': 'success', 'message': response})
        except zmq.Again:
            return jsonify({'status': 'error', 'message': 'Request timed out'}), 500
        except zmq.ZMQError as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500
    else:
        return jsonify({'status': 'error', 'message': 'Invalid command'}), 400

@app.route('/video_feed')
def video_feed():
    """Stream video from the camera."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames():
    """Generate video frames from the camera."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not start video capture.")
        return fallback_image()

    while True:
        success, frame = cap.read()
        if not success:
            return fallback_image()

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()

def fallback_image():
    """Generate a fallback 'No Signal' image."""
    no_signal_img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(no_signal_img, "No Signal", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
    ret, buffer = cv2.imencode('.jpg', no_signal_img)
    frame = buffer.tobytes()
    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def receive_logs():
    """Continuously receive logs."""
    while True:
        try:
            log_message = logs_socket.recv_string()
            received_logs.append(log_message)
            if len(received_logs) > 30:
                received_logs.pop(0)
        except zmq.Again:
            continue
        except zmq.ZMQError as e:
            print(f"ZeroMQ error: {e}")
            break

log_thread = threading.Thread(target=receive_logs, daemon=True)
log_thread.start()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
