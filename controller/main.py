from flask import Flask, request, jsonify, render_template, Response
import zmq
import os
import sys
import cv2
import socket
import concurrent.futures
import threading
import time
import numpy as np
import nmap

app = Flask(__name__)

NETWORK = '192.168.137.'
START_IP = 2
END_IP = 254
REQUIRED_PORTS = [5001, 5002]  # Both ports must be open

SERVER_IP = None  # Track the current server IP dynamically
context = zmq.Context()

# REQ socket for sending commands
socket = context.socket(zmq.PUB)
socket.setsockopt(zmq.RCVTIMEO, 2000)

# SUB socket for receiving logs
logs_socket = context.socket(zmq.SUB)
logs_socket.setsockopt_string(zmq.SUBSCRIBE, "")

received_logs = []  # Store received logs

def scan_ip_with_nmap(ip):
    """Use Nmap to scan a specific IP for required ports."""
    global SERVER_IP  # To update from within threads

    if SERVER_IP:  # Stop scanning if IP has already been found
        return None

    nm = nmap.PortScanner()
    ports_range = ','.join(map(str, REQUIRED_PORTS))

    print(f"Scanning {ip} for ports {ports_range}...")
    nm.scan(ip, ports_range)

    # Check if both required ports are open
    if all(nm[ip]['tcp'][port]['state'] == 'open' for port in REQUIRED_PORTS):
        print(f"IP {ip} has both required ports open.")
        SERVER_IP = ip  # Set found_ip and stop further scans
        return ip

    return None

def scan_network_with_nmap():
    """Scan the network and stop when a suitable IP is found."""
    for i in range(START_IP, END_IP + 1):
        ip = f"{NETWORK}{i}"
        open_ip = scan_ip_with_nmap(ip)
        if open_ip:
            print(f"Found IP: {open_ip}. Stopping further scans.")
            return open_ip  # Stop scanning and return the found IP

    print("No suitable IPs found.")
    return None

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
def scan_network():
    """Scan the network and connect to the first available server."""
    ip = scan_network_with_nmap()
    if ip and connect_to_server(ip):
        return jsonify({'status': 'success', 'message': f'Connected to {ip}'})
    else:
        print("No suitable server found.")
        return jsonify({'status': 'error', 'message': 'No suitable server found'}), 404

@app.route('/send_command', methods=['POST'])
def send_command():
    """Send commands to the ROS 2 node."""
    command = request.json.get('command')
    if command in ['start', 'stop', 'forward', 'backward', 'left', 'right', 'default']:
        try:
            socket.send_string(command)
            # response = socket.recv_string()
            return jsonify({'status': 'success', 'message': "response ok"})
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
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not start video capture.")
        yield from fallback_image()  # Use yield from for fallback

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            yield from fallback_image()
            break

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
