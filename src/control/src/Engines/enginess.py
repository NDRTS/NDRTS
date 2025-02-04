#! /usr/bin/env python3
import serial
import rospy
from std_msgs.msg import Float32
from messageconverter import MessageConverter

# Funcții pentru controlul direcției și vitezei mașinii

# Generează o comandă de control al vitezei
def get_speed_control_command(speed):
    return message_converter.get_command("1", speed=float(speed))

# Generează o comandă de control al direcției
def get_steer_control_command(steer_angle):
    return message_converter.get_command("2", steerAngle=float(steer_angle))

# Conexiune serială cu placa Nucleo
s = serial.Serial("/dev/ttyACM0", 19200)
message_converter = MessageConverter()
    
# Callback pentru comanda de direcție
def direction_command_callback(data):
    steer_angle = data.data
    steer_command = get_steer_control_command(steer_angle)
    send_serial_command(steer_command)

# Callback pentru comanda de viteză
def speed_command_callback(data):
    speed = data.data
    speed_command = get_speed_control_command(speed)
    send_serial_command(speed_command)

# Funcție pentru trimiterea comenzilor prin serial
def send_serial_command(command):
    s.write(command.encode())

# Funcția principală de inițializare a nodului ROS
def main():
    rospy.init_node('direction_and_speed_command_subscriber', anonymous=True)
    rospy.Subscriber("/direction_lane", Float32, direction_command_callback)
    rospy.Subscriber("/speed", Float32, speed_command_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
