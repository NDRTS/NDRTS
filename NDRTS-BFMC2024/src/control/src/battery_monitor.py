#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32
from messageconverter import MessageConverter

# === Serial setup ===
ser = serial.Serial("/dev/ttyACM0", 19200, timeout=1)  # Match baudrate with Nucleo
message_converter = MessageConverter()

# === ROS publisher ===
battery_pub = None

# === Send enable command for battery ===
def send_enable_command():
    cmd = message_converter.get_command("5", activate=True)
    rospy.loginfo(f"Sending enable battery command: {cmd.strip()}")
    ser.write(cmd.encode())

# === Main read loop ===
def read_battery_loop():
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        line = ser.readline().decode().strip()
        if "@5:" in line:
            try:
                value_str = line.split("@5:")[1].split(";")[0]
                battery_level = float(value_str)
                battery_pub.publish(battery_level)
            except Exception as e:
                rospy.logwarn(f"Error parsing battery data: {line} -> {e}")
        rate.sleep()


# === Main ROS node ===
def main():
    global battery_pub

    rospy.init_node("battery_monitor_node", anonymous=True)
    battery_pub = rospy.Publisher("/battery", Float32, queue_size=10)

    send_enable_command()
    read_battery_loop()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
