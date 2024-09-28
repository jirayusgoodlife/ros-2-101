# zmq_server.py
import zmq

def main():
    # Create a ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    socket = context.socket(zmq.REP)
    socket.bind("tcp://0.0.0.0:5555")  # Bind to port 5555 on all available interfaces

    print("Server is listening on port 5555...")

    while True:
        # Wait for the next request from the client
        message = socket.recv_string()
        print(f"Received request: {message}")

        # Process the message and send a response
        if message == "status":
            response = "Server is running and responsive."
        elif message == "ping":
            response = "pong"
        else:
            response = f"Unknown command: {message}"

        # Send the response back to the client
        socket.send_string(response)

if __name__ == "__main__":
    main()
