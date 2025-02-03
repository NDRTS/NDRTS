from messageconverter import MessageConverter
import serial
import time
from control import *

#Car steering and speed control
def get_speed_control_command(speed):
    return message_converter.get_command("1", speed=float(speed))
def get_steer_control_command(steer_angle):
    return message_converter.get_command("2", steerAngle=float(steer_angle))

#Serial Connection to the Nucleo Board
s = serial.Serial("/dev/ttyACM0", 19200)
message_converter = MessageConverter()
def START():
    s.write(get_speed_control_command(10).encode('utf-8'))
    s.write(get_steer_control_command(0).encode('utf-8'))

def STOP_SIGN():
    s.write(get_steer_control_command(0).encode('utf-8'))
    s.write(get_speed_control_command(0).encode('utf-8'))
    time.sleep(10)
    s.write(get_speed_control_command(10).encode('utf-8'))
    print("STOPPED")

def CROSSWALK_WITHOUT_PEDESTRIAN():
    print("NO PEDESTRIAN")
    s.write(get_speed_control_command(10).encode('utf-8'))
    # s.write(get_speed_control_command(10).encode('utf-8'))

def PEDESTRIAN():
    s.write(get_speed_control_command(0).encode('utf-8'))

def NON_PEDESTRIAN():
    s.write(get_speed_control_command(10).encode('utf-8'))

def CROSSWALK_WITH_PEDESTRIAN():
    print("PEDESTRIAN")
    s.write(get_speed_control_command(0).encode('utf-8'))
    

def HIGHWAY_ENTER():
    s.write(get_speed_control_command(40).encode('utf-8'))

def HIGHWAY_EXIT():
    s.write(get_speed_control_command(20).encode('utf-8'))

def PRIORITY_SIGN():
    s.write(get_speed_control_command(10).encode('utf-8'))

def ROUNDABOUTSIGN():
    speed = get_speed_control_command(10)
    s.write(speed.encode('utf-8'))
    steer = get_steer_control_command(0)
    s.write(steer.encode('utf-8'))
    time.sleep(1.5)
    steer = get_steer_control_command(25.0)
    s.write(steer.encode('utf-8'))
    time.sleep(4.5)   # 5.5
    steer = get_steer_control_command(-15.0)
    s.write(steer.encode('utf-8'))
    time.sleep(1)
    steer = get_steer_control_command(-25.0)
    s.write(steer.encode('utf-8'))
    time.sleep(19)
    steer = get_steer_control_command(25.0)
    s.write(steer.encode('utf-8'))
    time.sleep(3)
    speed = get_speed_control_command(10)
    s.write(speed.encode('utf-8'))

def RED():
    s.write(get_speed_control_command(0).encode('utf-8'))
    time.sleep(7.0)
    s.write(get_speed_control_command(10).encode('utf-8'))

def GREEN():
        s.write(get_speed_control_command(10).encode('utf-8'))


def PARKING_SPACE_1():
    steer = get_steer_control_command(0)
    time.sleep(0.2)
    speed = get_speed_control_command(-5)
    s.write(speed.encode('utf-8'))
    steer = get_steer_control_command(25.0)
    s.write(steer.encode('utf-8'))
    time.sleep(7)
    steer = get_steer_control_command(-25.0)
    s.write(steer.encode('utf-8'))
    time.sleep(7.5)
    steer = get_steer_control_command(0)
    s.write(steer.encode('utf-8'))
    speed = get_speed_control_command(5)
    s.write(speed.encode('utf-8'))
    time.sleep(2)
    speed = get_speed_control_command(0)
    s.write(speed.encode('utf-8'))
    time.sleep(5)
#Parcare Iesire
    speed = get_speed_control_command(5)
    s.write(speed.encode('utf-8'))
    steer = get_steer_control_command(-25.0)
    s.write(steer.encode('utf-8'))
    time.sleep(7)
    steer = get_steer_control_command(25.0)
    s.write(steer.encode('utf-8'))
    time.sleep(9.5)
    speed = get_speed_control_command(10)
    s.write(speed.encode('utf-8'))


#     s.write(get_speed_control_command(-10).encode('utf-8'))
#     print("PARKED")

# def PARKING_SPACE_2():
#     s.write(get_speed_control_command(10).encode('utf-8'))
#     time.sleep(20)
#     s.write(get_steer_control_command(-25).encode('utf-8'))
#     s.write(get_speed_control_command(-10).encode('utf-8'))
#     time.sleep(5)
#     s.write(get_steer_control_command(25).encode('utf-8'))
#     s.write(get_speed_control_command(-10).encode('utf-8'))
#     print("PARKED")




