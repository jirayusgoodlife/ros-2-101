import zmq
import cv2
import numpy as np

def main():
    # Setup ZeroMQ SUB socket
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    # 👇 Drop old frames
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt(zmq.RCVHWM, 1)
    subscriber.connect("tcp://192.168.137.101:5003")  # Change if running on different host
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

    print("Connected to camera stream. Waiting for frames...")

    try:
        while True:
            # Receive the JPEG frame as bytes
            frame_bytes = subscriber.recv()
            
            # Convert bytes to numpy array
            nparr = np.frombuffer(frame_bytes, np.uint8)

            # Decode image
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                cv2.imshow("Camera Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Failed to decode frame.")
    except KeyboardInterrupt:
        print("Client interrupted by user.")
    finally:
        subscriber.close()
        context.term()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
