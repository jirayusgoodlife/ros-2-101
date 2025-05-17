from flask import Flask, request, jsonify, render_template, Response
from flask_socketio import SocketIO
import zmq
import os
import sys
import cv2
import socket
import concurrent.futures
from threading import Lock, Thread
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

context = zmq.Context()

# REQ socket for sending commands
socket = context.socket(zmq.PUB)
socket.setsockopt(zmq.RCVTIMEO, 2000)

# SUB socket for receiving logs
logs_socket = context.socket(zmq.SUB)
logs_socket.setsockopt_string(zmq.SUBSCRIBE, "")


# Global thread state
stream_thread = None
stream_thread_lock = Lock()
streaming_ip = None
stop_stream_flag = False
SERVER_IP = '192.168.137.101'  # Replace with your actual default IP

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
    
def get_connecting_frame():
    no_signal_img = np.zeros((480, 640, 3), dtype=np.uint8)
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(no_signal_img, "Waiting to connect", (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 255), 2)
    cv2.putText(no_signal_img, current_time, (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 255), 2)
    ret, buffer = cv2.imencode('.jpg', no_signal_img)
    frame_bytes = buffer.tobytes()
    socketio.emit('video_frame', frame_bytes)

def video_stream_thread(server_ip):
    global stop_stream_flag
    stop_stream_flag = False

    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt(zmq.RCVHWM, 1)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    subscriber.connect(f"tcp://{server_ip}:5003")

    get_connecting_frame()
    socketio.sleep(0.5)

    # Drain any old buffered frames
    for _ in range(5):
        try:
            subscriber.recv(zmq.NOBLOCK)
        except zmq.Again:
            break

    while not stop_stream_flag:
        try:
            frame_bytes = subscriber.recv(zmq.NOBLOCK)  # Non-blocking read
            socketio.emit('video_frame', frame_bytes)
            socketio.sleep(0.5) #   ถ้า delay แก้เวลา
        except zmq.Again:
            socketio.sleep(0.01)
        except zmq.ZMQError as e:
            app.logger.error(f"Streaming error: {e}")
            break

    subscriber.close()
    context.term()

    
@socketio.on('start_stream')
def handle_start_stream():
    global stream_thread, streaming_ip
    server_ip = SERVER_IP #data.get("server_ip", SERVER_IP)

    with stream_thread_lock:
        if stream_thread and stream_thread.is_alive() and streaming_ip == server_ip:
            return  # Already running

        stop_stream_flag = False
        streaming_ip = server_ip
        stream_thread = socketio.start_background_task(video_stream_thread, server_ip)

@socketio.on('restart_stream')
def handle_restart_stream():
    global stream_thread, streaming_ip, stop_stream_flag
    server_ip = SERVER_IP #data.get("server_ip", SERVER_IP)

    with stream_thread_lock:
        if stream_thread and stream_thread.is_alive():
            stop_stream_flag = True
            stream_thread.join(timeout=2)

        stop_stream_flag = False
        streaming_ip = server_ip
        stream_thread = socketio.start_background_task(video_stream_thread, server_ip)

    socketio.emit("stream_status", {"status": "restarted", "ip": server_ip})
    
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

log_thread = Thread(target=receive_logs, daemon=True)
log_thread.start()

if __name__ == '__main__':
    # app.run(host='0.0.0.0', port=5000, debug=True)
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
