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

# Initialize control variables
speed = 0      # Current speed
steering = 0   # Current steering angle in degrees (range: -16 to 16)

print("Script started. Use W/A/S/D to control speed/steering, X/C to set low speeds,")
print("B to stop lane-keeping, N to resume lane-keeping, and Q to quit.")

try:
    while not rospy.is_shutdown():
        key = get_key()

        # Adjust speed and steering based on key press
        if key == 'w':      # Increase speed by 10
            speed += 10
        elif key == 's':    # Decrease speed by 10
            speed -= 10
        elif key == 'a':    # Turn left: decrease steering by 2 degrees, limit to -16
            steering = max(steering - 5, -40)
        elif key == 'd':    # Turn right: increase steering by 2 degrees, limit to 16
            steering = min(steering + 5, 40)
        elif key == 'x':    # Set low speed to 10
            speed = 10
        elif key == 'c':    # Set low speed to 20
            speed = 20
        elif key == 'b':    # Stop lane-keeping
            stop_lanekeeping_pub.publish(1)
            print("[INFO] stop_lanekeeping sent.")
        elif key == 'n':    # Resume lane-keeping
            stop_lanekeeping_pub.publish(0)
            print("[INFO] start_lanekeeping sent.")
        elif key == 'q':    # Quit the script
            print("Exiting script.")
            break

        # Publish values to ROS topics
        speed_pub.publish(speed)
        steer_pub.publish(steering)

        # Print current state for debugging
        print(f"[DEBUG] Speed: {speed}, Steering: {steering}")

        # Sleep briefly to prevent high CPU usage
        rospy.sleep(0.1)

except Exception as e:
    print("Exception:", e)

print("Script stopped.")
