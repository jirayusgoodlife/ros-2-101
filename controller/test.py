# zmq_client.py
import zmq

def main():
    # Create a ZeroMQ context
    context = zmq.Context()

    # Create a REQ (request) socket
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://192.168.137.1:5555")  # Connect to the server on localhost at port 5555

    # List of commands to send to the server
    commands = ["status", "ping", "hello"]

    for command in commands:
        print(f"Sending command: {command}")
        
        # Send the command to the server
        socket.send_string(command)

        # Receive the response from the server
        response = socket.recv_string()
        print(f"Received response: {response}")

if __name__ == "__main__":
    main()
