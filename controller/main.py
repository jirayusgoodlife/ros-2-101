from flask import Flask, request, jsonify, render_template, Response
from flask_socketio import SocketIO
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
import datetime
import base64

app = Flask(__name__)
socketio = SocketIO(app)

NETWORK = '192.168.137.'
START_IP = 100
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


stream_thread = None
streaming_ip = None
stream_thread_lock = threading.Lock()

received_logs = []  # Store received logs

def scan_ip_with_nmap(ip):
    """Use Nmap to scan a specific IP for required ports."""
    global SERVER_IP

    if SERVER_IP:  # Stop scanning if IP is found
        return None

    nm = nmap.PortScanner()
    ports_range = ','.join(map(str, REQUIRED_PORTS))

    app.logger.info(f"Scanning {ip} for ports {ports_range}...")
    nm.scan(ip, ports_range)

    # Check if the IP is present in the scan result
    if ip not in nm.all_hosts():
        app.logger.info(f"IP {ip} not found in the scan result.")
        return None

    # Check if both required ports are open
    if all(nm[ip]['tcp'][port]['state'] == 'open' for port in REQUIRED_PORTS):
        app.logger.info(f"IP {ip} has both required ports open.")
        SERVER_IP = ip
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

    app.logger.info("No suitable IPs found.")
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
    if command in ['start', 'stop', 'forward', 'backward', 'left', 'right', 'default', 'mode_delay_on', 'mode_delay_off']:
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
    
subscriber = None
def video_stream_thread(server_ip):
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt(zmq.RCVHWM, 1)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    subscriber.connect(f"tcp://{server_ip}:5003")

    # Emit fallback "Connecting" frame first
    get_connecting_frame()

    # Allow the client to see the fallback before real stream
    socketio.sleep(1)

    while True:
        try:
            frame_bytes = subscriber.recv()
            socketio.emit('video_frame', frame_bytes)  # Send raw JPEG binary
            socketio.sleep(0.03)  # ~30 FPS
        except zmq.ZMQError as e:
            app.logger.error(f"Streaming error: {e}")
            break


@socketio.on('start_stream')
def handle_start_stream(data):
    global stream_thread, streaming_ip

    with stream_thread_lock:
        if stream_thread and streaming_ip == SERVER_IP:
            return  # Already streaming

        # Emit fallback frame immediately while stream thread is starting
        get_no_signal_frame()

        # Set IP and start background task
        streaming_ip = SERVER_IP
        stream_thread = socketio.start_background_task(video_stream_thread, SERVER_IP)


            
def get_no_signal_frame():
    no_signal_img = np.zeros((480, 640, 3), dtype=np.uint8)
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    text = f"No Signal {current_time}"
    app.logger.error(text)
    cv2.putText(no_signal_img, text, (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Convert image to JPEG bytes
    ret, buffer = cv2.imencode('.jpg', no_signal_img)
    frame_bytes = buffer.tobytes()

    # Emit raw JPEG bytes (same format as actual frames)
    socketio.emit('video_frame', frame_bytes)
    
def get_connecting_frame():
    no_signal_img = np.zeros((480, 640, 3), dtype=np.uint8)
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    cv2.putText(no_signal_img, "Waiting to connect", (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 255), 2)

    cv2.putText(no_signal_img, current_time, (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 255), 2)
    
    # Encode image to JPEG
    ret, buffer = cv2.imencode('.jpg', no_signal_img)
    frame_bytes = buffer.tobytes()

    # Emit the JPEG frame as binary
    socketio.emit('video_frame', frame_bytes)
                
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(SERVER_IP), mimetype='multipart/x-mixed-replace; boundary=frame')


def generate_frames(server_ip):
    context = zmq.Context()
    camera_socket = None

    while True:
        if server_ip is None:
            yield from fallback_image_with_time()
            continue

        try:
            if camera_socket is None:
                camera_socket = context.socket(zmq.SUB)
                camera_socket.setsockopt(zmq.CONFLATE, 1)
                camera_socket.setsockopt(zmq.RCVHWM, 1)
                camera_socket.setsockopt_string(zmq.SUBSCRIBE, "")
                camera_socket.connect(f"tcp://{server_ip}:5003")
                app.logger.info(f"Connected to camera feed at tcp://{server_ip}:5003")

            try:
                frame_bytes = camera_socket.recv(flags=zmq.NOBLOCK)
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
                )
            except zmq.Again:
                # No frame ready yet, wait a bit
                time.sleep(0.01)
                continue
        except Exception as e:
            app.logger.error(f"Camera stream error: {e}")
            yield from fallback_image_with_time()
            if camera_socket:
                camera_socket.close()
                camera_socket = None
            time.sleep(1)

    if camera_socket:
        camera_socket.close()
    context.term()

    
def fallback_image_with_time():
    """Generate a fallback 'No Signal' image with the current time."""
    no_signal_img = np.zeros((480, 640, 3), dtype=np.uint8)
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    text = f"No Signal {current_time}"
    app.logger.error(text)
    cv2.putText(no_signal_img, text, (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    ret, buffer = cv2.imencode('.jpg', no_signal_img)
    frame = buffer.tobytes()
    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

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
    # app.run(host='0.0.0.0', port=5000, debug=True)
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
