#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from signs import PARALLEL_PARK_LEFT
import sys, termios, tty
import signal
import time

def get_key():
    """Reads a single keypress from stdin (non-blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # read one character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def signal_handler(sig, frame):
    print("\n🛑 Exiting manual parking trigger...")
    rospy.signal_shutdown("Keyboard interrupt")
    sys.exit(0)

def main():
    rospy.init_node('manual_parking_trigger', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    print("🚗 Press 'p' to start the PARALLEL_PARK_LEFT() maneuver.")
    print("❌ Press 'q' to quit.")

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'p':
            time.sleep(0.5)  # small delay to ensure subscribers process the message
            print("🔁 Triggering parallel parking...")
            PARALLEL_PARK_LEFT()
        elif key == 'q':
            print("👋 Quitting...")
            break
        time.sleep(0.1)

if __name__ == '__main__':
    main()
