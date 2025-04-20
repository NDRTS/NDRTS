# import rospy
# from std_msgs.msg import String

# import serial
# import time
# import datetime
# import socket

# # Socket server details
# # server_ip = "192.168.86.97"
# # server_port = 4023

# # # Create a socket
# # server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# # server_socket.bind((server_ip, server_port))
# # server_socket.listen(1)

# # Initialize ROS node
# rospy.init_node('gps_publisher')

# # Create publisher for /gps_data topic
# gps_pub = rospy.Publisher('/gps_data', String, queue_size=10)

# DWM = serial.Serial(port="/dev/ttyACM1", baudrate=115200)
# print("Connected to " + DWM.name)
# DWM.write("\r\r".encode())
# time.sleep(1)
# DWM.write("lec\r".encode())
# time.sleep(1)

# try:
#     while not rospy.is_shutdown():
#         start_time = time.time()  # Record start time for each loop iteration
#         line = DWM.readline()
#         if line:
#             if len(line) >= 140:
#                 parse = line.decode().split(",")
#                 if "POS" in parse and len(parse) >= parse.index("POS") + 3:
#                     try:
#                         x_pos = float(parse[parse.index("POS") + 1])
#                         y_pos = float(parse[parse.index("POS") + 2])
#                         # Ensure the X and Y position values are valid
#                         if not (x_pos == float('nan') or y_pos == float('nan')):
#                             print("Time:", datetime.datetime.now().strftime("%H:%M:%S"), "X:", x_pos, ",Y:", y_pos)
#                             # Publish position data to ROS topic
#                             gps_pub.publish(f"{x_pos},{y_pos}")
#                             socketdata = f"X: {x_pos} Y: {y_pos}"
#                             client_socket, _ = server_socket.accept()
#                             client_socket.sendall(socketdata.encode())
#                             client_socket.close()
#                     except ValueError:
#                         pass  # Skip publishing or printing invalid data
#         # Calculate elapsed time and sleep if necessary to maintain the rate of 2 reads per second
#         time_elapsed = time.time() - start_time
#         if time_elapsed < 0.0:
#             time.sleep(0.0 - time_elapsed)
# except Exception as ex:
#     print(ex)
# finally:
#     DWM.write("\r".encode())
#     DWM.close()

# import rospy
# from std_msgs.msg import String

# import serial
# import time
# import datetime
# import socket
# import threading

# # Socket server details
# server_ip = "192.168.86.97"
# server_port = 3905

# # Create a socket
# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind((server_ip, server_port))
# server_socket.listen(5)

# # Initialize ROS node
# rospy.init_node('gps_publisher')

# # Create publisher for /gps_data topic
# gps_pub = rospy.Publisher('/gps_data', String, queue_size=10)

# DWM = serial.Serial(port="/dev/ttyACM1", baudrate=115200)
# print("Connected to " + DWM.name)
# DWM.write("\r\r".encode())
# time.sleep(1)
# DWM.write("lec\r".encode())
# time.sleep(1)

# # Function to handle client connections
# def handle_client(client_socket):
#     try:
#         while True:
#             line = DWM.readline()
#             if line:
#                 if len(line) >= 140:
#                     parse = line.decode().split(",")
#                     if "POS" in parse and len(parse) >= parse.index("POS") + 3:
#                         try:
#                             x_pos = float(parse[parse.index("POS") + 1])
#                             y_pos = float(parse[parse.index("POS") + 2])
#                             # Ensure the X and Y position values are valid
#                             if not (x_pos == float('nan') or y_pos == float('nan')):
#                                 print("Time:", datetime.datetime.now().strftime("%H:%M:%S"), "X:", x_pos, ",Y:", y_pos)
#                                 # Publish position data to ROS topic
#                                 gps_pub.publish(f"{x_pos},{y_pos}")
#                                 # Send data over the socket
#                                 socketdata = f"X: {x_pos} Y: {y_pos}"
#                                 client_socket.sendall(socketdata.encode())
#                         except ValueError:
#                             pass  # Skip publishing or printing invalid data
#     except Exception as ex:
#         print("Error:", ex)
#     finally:
#         client_socket.close()

# try:
#     while not rospy.is_shutdown():
#         client_socket, _ = server_socket.accept()
#         print("Connected to client")
#         client_thread = threading.Thread(target=handle_client, args=(client_socket,))
#         client_thread.start()
# except Exception as ex:
#     print("Error:", ex)
# finally:
#     DWM.write("\r".encode())
#     DWM.close()

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
