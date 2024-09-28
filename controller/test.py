import zmq

# Set up ZeroMQ REQ socket to communicate with the RobotController node
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.137.1:5555")  # Replace <ROS_NODE_IP> with the correct IP address

try:
    # Send the status request
    socket.send_string("status")

    # Receive the response from the ROS 2 node
    response = socket.recv_string()
    print(f"Received response: {response}")

except zmq.ZMQError as e:
    print(f"ZeroMQ Error: {e}")

finally:
    # Properly close the socket and context
    socket.close()
    context.term()
