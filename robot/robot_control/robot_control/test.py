import curses
from robot import Robot

def main(stdscr):
    robot = Robot()

    # Clear the screen and hide the cursor
    stdscr.clear()
    curses.curs_set(0)

    # Print controls to the curses window
    display_controls(stdscr)

    try:
        while True:
            key = stdscr.getkey().lower()

            if key == "w":
                robot.move_forward()
                stdscr.addstr(12, 0, "Moving forward...             ")
            elif key == "s":
                robot.move_backward()
                stdscr.addstr(12, 0, "Moving backward...            ")
            elif key == "a":
                robot.turn_left()
                stdscr.addstr(12, 0, "Turning left...               ")
            elif key == "d":
                robot.turn_right()
                stdscr.addstr(12, 0, "Turning right...              ")
            elif key == "t":
                robot.start()
                stdscr.addstr(12, 0, "Robot started.                ")
            elif key == "y":
                robot.stop()
                stdscr.addstr(12, 0, "Robot stopped.                ")
            elif key == "f":
                robot.position_default()
                stdscr.addstr(12, 0, "Resetting to default position.")
            elif key == "q":
                stdscr.addstr(12, 0, "Exiting...                    ")
                break
            else:
                stdscr.addstr(12, 0, "Invalid command. Try again.   ")

            # Refresh the screen to show updates
            stdscr.refresh()

    except Exception as e:
        stdscr.addstr(14, 0, f"Error: {e}")
        stdscr.refresh()
        robot.reset()
        stdscr.addstr(15, 0, "Robot reset and program terminated.")
        stdscr.refresh()
        stdscr.getch()  # Wait for a key press to exit

def display_controls(stdscr):
    controls = [
        "Control your robot:",
        "w: Move Forward",
        "s: Move Backward",
        "a: Turn Left",
        "d: Turn Right",
        "t: Start",
        "y: Stop",
        "f: Reset to Default Position",
        "q: Quit"
    ]
    for i, control in enumerate(controls):
        stdscr.addstr(i, 0, control)
    stdscr.refresh()

def get_servo_inputs(stdscr):
    stdscr.addstr(16, 0, "Enter servo: ")
    servo = stdscr.getstr(16, 13, 5).decode("utf-8")
    stdscr.addstr(17, 0, "Enter low limit: ")
    low = stdscr.getstr(17, 16, 5).decode("utf-8")
    stdscr.addstr(18, 0, "Enter high limit: ")
    high = stdscr.getstr(18, 17, 5).decode("utf-8")
    return servo, low, high

def get_servo_position_input(stdscr):
    stdscr.addstr(19, 0, "Enter servo ID: ")
    servo_id = stdscr.getstr(19, 15, 5).decode("utf-8")
    stdscr.addstr(20, 0, "Enter position: ")
    position = stdscr.getstr(20, 15, 5).decode("utf-8")
    return servo_id, position

curses.wrapper(main)
