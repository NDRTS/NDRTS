###################################################### Varianta Finala, cu implementarea de trimitere date pt Dash separata
import rospy
from std_msgs.msg import String
import serial
import time
import datetime

# Initialize ROS node
rospy.init_node('gps_publisher')

# Create publisher for /gps_data topic
gps_pub = rospy.Publisher('/gps_data', String, queue_size=10)

DWM = serial.Serial(port="/dev/ttyACM1", baudrate=115200)
print("Connected to " + DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)

while not rospy.is_shutdown():
    line = DWM.readline()
    if line:
        if len(line) >= 100:
            parse = line.decode().split(",")
            if "POS" in parse and len(parse) >= parse.index("POS") + 3:
                try:
                    x_pos = float(parse[parse.index("POS") + 1])
                    y_pos = float(parse[parse.index("POS") + 2])
                    # Ensure the X and Y position values are valid
                    if not (x_pos == float('nan') or y_pos == float('nan')):
                        print("Time:", datetime.datetime.now().strftime("%H:%M:%S"), "X:", x_pos, ",Y:", y_pos)
                        # Publish position data to ROS topic
                        time.sleep(0.1)
                        gps_pub.publish(f"{x_pos},{y_pos}")
                except ValueError:
                    pass  # Skip publishing or printing invalid data

DWM.write("\r".encode())
DWM.close()
