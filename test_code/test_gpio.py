from pylx16a.lx16a import *
import time

# Initialize the LX16A communication
LX16A.initialize("/dev/ttyUSB0")

# Define servos with ID, min, max, low, high, and default angles
servos = {
    1: {"servo": LX16A(1), "min_angle": 30, "max_angle": 155,  "low": 90, "high": 150, "default": 90},
    2: {"servo": LX16A(2), "min_angle": 150, "max_angle": 210, "low": 160, "high": 190, "default": 160},
    3: {"servo": LX16A(3), "min_angle": 30, "max_angle": 100, "low": 90, "high": 40, "default": 90},
    4: {"servo": LX16A(4), "min_angle": 80, "max_angle": 200, "low": 150, "high": 80, "default": 150},
    5: {"servo": LX16A(5), "min_angle": 140, "max_angle": 210, "low": 160, "high": 240, "default": 160},
    6: {"servo": LX16A(6), "min_angle": 40, "max_angle": 110, "low": 100, "high": 50, "default": 100},
    7: {"servo": LX16A(7), "min_angle": 80, "max_angle": 210, "low": 190, "high": 90, "default": 155},
    8: {"servo": LX16A(8), "min_angle": 31, "max_angle": 120, "low": 110, "high": 60, "default": 80},
    9: {"servo": LX16A(9), "min_angle": 120, "max_angle": 170, "low": 130, "high": 160, "default": 145},
    10: {"servo": LX16A(10), "min_angle": 30, "max_angle": 170, "low": 80, "high": 170, "default": 80},
    11: {"servo": LX16A(11), "min_angle": 25, "max_angle": 90, "low": 90, "high": 25, "default": 70},
    12: {"servo": LX16A(12), "min_angle": 130, "max_angle": 200, "low": 150, "high": 190, "default": 150},
}

# Set the angle limits for all servos
for servo_data in servos.values():
    servo = servo_data["servo"]
    servo.set_angle_limits(servo_data["min_angle"], servo_data["max_angle"])

# Function to move a servo to a given position
def move_servo(servo_id, position):
    servo_data = servos.get(servo_id)
    if servo_data:
        min_angle = servo_data["min_angle"]
        max_angle = servo_data["max_angle"]
        
        # Validate position is within the servo's angle limits
        if min_angle <= position <= max_angle:
            servo_data["servo"].move(position)
            print(f"Moved servo {servo_id} to position {position}")
        else:
            print(f"Invalid position! Please enter a value between {min_angle} and {max_angle}.")
    else:
        print(f"Invalid servo ID: {servo_id}")

# Function to set all servos to their default positions
def set_default_positions():
    for servo_id, servo_data in servos.items():
        default_position = servo_data["default"]
        print(f"Setting servo {servo_id} to default position: {default_position}")
        move_servo(servo_id, default_position)
        time.sleep(1)  # Wait for 1 second for smooth transitions

# Main loop for user input
try:
    while True:
        # Call the function to set all servos to their default positions
        print("Moving all servos to their default positions...")
        set_default_positions()
        time.sleep(5)  # Wait for 5 seconds before repeating

except KeyboardInterrupt:
    print("\nExiting...")
