import rospy
from std_msgs.msg import Float32, Int32
import sys
import termios
import tty

# Initialize the ROS node
rospy.init_node('speed_and_steer_controller', anonymous=True)
speed_pub = rospy.Publisher('/speed', Float32, queue_size=10)
steer_pub = rospy.Publisher('/direction_lane', Float32, queue_size=10)
stop_lanekeeping_pub = rospy.Publisher('/stop_lanekeeping', Int32, queue_size=10)

# Function to read a single keypress from the terminal
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# Variables for control
speed = 0
steering = 0

try:
    print("Script started. Use W/A/S/D for control. Press Q to quit.")
    while not rospy.is_shutdown():
        key = get_key()

        # Handle key presses for speed and steering
        if key == 'w':  # Increase speed
            speed = 10
        elif key == 's':  # Decrease speed
            speed = -10
        elif key == 'a':  # Steer left
            steering = -10
        elif key == 'd':  # Steer right
            steering = 10
        elif key == 'x':  # Set low speed
            speed = 10
        elif key == 'c':  # Set low speed
            speed = 20
        elif key == 'y':  # Set maximum speed
            speed = 50
        elif key == 'b':  # Stop lane-keeping
            stop_lanekeeping_pub.publish(1)
            print("[INFO] stop_lanekeeping = 1 sent.")
        elif key == 'n':  # Resume lane-keeping
            stop_lanekeeping_pub.publish(0)
            print("[INFO] stop_lanekeeping = 0 sent.")
        elif key == 'q':  # Quit
            print("Exiting script.")
            break

        # Reset speed or steering when certain keys are released
        if key in ['w', 's']:  # Stop movement
            speed = 0
        if key in ['a', 'd']:  # Stop steering
            steering = 0

        # Publish values to ROS topics
        speed_pub.publish(speed)
        if abs(steering) > 3.5:  # Publish steering only if significant
            steer_pub.publish(steering)

        # Print current state for debugging
        print(f"[DEBUG] Speed: {speed}, Steering: {steering}")

        # Pause to prevent high CPU usage
        rospy.sleep(0.1)

except Exception as e:
    print("Exception:", e)

# End script
print("Script stopped.")
