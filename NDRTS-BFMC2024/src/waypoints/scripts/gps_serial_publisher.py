#!/usr/bin/env python3
"""
Read serial frames coming from the DWM1001 tag (“POS,x,y,…”) and publish the
car position on /car_position             geometry_msgs/PointStamped
"""
import serial, time, rospy
from geometry_msgs.msg import PointStamped

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
SERIAL_PORT = rospy.get_param("~port", "/dev/ttyACM1")
BAUD        = rospy.get_param("~baud", 115200)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def send_lec(ser):
    """Put the DWM1001 in continuous location-engine mode."""
    ser.write(b"\r\r");  time.sleep(0.1)
    ser.write(b"lec\r"); time.sleep(0.1)

def open_serial():
    """(Re)open the USB-CDC port and initialise the tag. Retries forever."""
    while not rospy.is_shutdown():
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
            rospy.loginfo("Serial open on %s", SERIAL_PORT)
            send_lec(ser)
            return ser
        except serial.SerialException as e:
            rospy.logwarn("Cannot open %s (%s). Retrying in 2 s",
                          SERIAL_PORT, e)
            time.sleep(2)

def parse_line(line: str):
    """
    Return (x,y) when the frame contains a full  ‘…POS,<x>,<y>…’  sentence.
    Otherwise return None.
    """
    if "POS" not in line:
        return None

    parts = line.strip().split(",")              # drop CR/LF, split commas
    try:
        i = parts.index("POS")                   # find the keyword
        x, y = float(parts[i + 1]), float(parts[i + 2])
        return x, -y                             # flip Y once for SVG
    except (ValueError, IndexError):
        # either the floats aren’t there yet or can’t be parsed -> ignore
        return None


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    rospy.init_node("car_position_publisher")
    pub  = rospy.Publisher("/car_position", PointStamped, queue_size=5)

    ser  = open_serial()                   # first connection
    rate = rospy.Rate(10)                  # 10 Hz

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode(errors="ignore")
        except serial.SerialException as e:
            rospy.logwarn("Serial error: %s — reopening port", e)
            ser.close()
            ser = open_serial()
            continue                       # skip this iteration

        xy = parse_line(line)
        if xy is None:
            rate.sleep()
            continue

        x, y = xy
        msg = PointStamped()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.point.x, msg.point.y = x, y
        pub.publish(msg)

        rospy.logdebug("car @ %.2f , %.2f", x, y)
        rate.sleep()

    # clean exit
    try:
        ser.write(b"\r")
        ser.close()
    except Exception:
        pass

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
