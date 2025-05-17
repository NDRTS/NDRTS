from messageconverter import MessageConverter
import serial
import time
from control import *
import rospy
from std_msgs.msg import Float32, Int32

# rospy.init_node('signs_node', anonymous=True)

speed_pub = rospy.Publisher('/speed', Float32, queue_size=10)
direction_pub = rospy.Publisher('/direction_lane', Float32, queue_size=10)
stop_lanekeeping_pub = rospy.Publisher('/stop_lanekeeping_cmd', Int32, queue_size=10)

#Car steering and speed control
def get_speed_control_command(speed):
    return message_converter.get_command("1", speed=float(speed))
def get_steer_control_command(steer_angle):
    return message_converter.get_command("2", steerAngle=float(steer_angle))

def set_speed(speed):
    speed_pub.publish(speed)

def set_steering(angle):
    direction_pub.publish(angle)

#Serial Connection to the Nucleo Board
s = serial.Serial("/dev/ttyACM0", 19200)
message_converter = MessageConverter()
def START():
    set_speed(10)
    set_steering(0)

def STOP_SIGN():
    set_steering(0)
    set_speed(0)
    time.sleep(10)
    set_speed(10)
    print("STOPPED")

def CROSSWALK_WITHOUT_PEDESTRIAN():
    print("NO PEDESTRIAN")
    set_speed(10)
    # set_speed(10)

def PEDESTRIAN():
    set_speed(0)

def NON_PEDESTRIAN():
    set_speed(10)

def CROSSWALK_WITH_PEDESTRIAN():
    print("PEDESTRIAN")
    set_speed(0)
    

def HIGHWAY_ENTER():
    set_speed(40)

def HIGHWAY_EXIT():
    set_speed(20)

def PRIORITY_SIGN():
    set_speed(10)

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
    set_speed(0)
    time.sleep(7.0)
    set_speed(10)

def GREEN():
    set_speed(10)

def PARALLEL_PARK_LEFT():
    print("🅿️ Initiating LEFT parallel parking maneuver")
    stop_lanekeeping_pub.publish(1)
    # Step 1: Stop and wait
    print("Step 1 - Stopping and preparing to park...")
    set_speed(0)
    set_steering(0)
    time.sleep(1)

    # Step 2: Start reversing with full left steer
    print("Step 2 - Reversing into the parking space...")
    set_speed(-5)  # reverse slowly
    set_steering(-20)  # steer full left (into the space)
    time.sleep(15)  # long enough to start rotating

    # Step 3: Briefly straighten out
    print("Step 3 - Straightening out...")
    set_steering(0)
    set_speed(-5)
    time.sleep(3)

    # Step 4: Steer right to align car in the box
    print("Step 4 - Aligning in the parking space...")
    set_steering(20)
    set_speed(-5)
    time.sleep(10)

    # Step 5: Steer right to align car in the box
    print("Step 5 - Aligning in the parking space...")
    set_steering(-20)
    set_speed(5)
    time.sleep(5)


    # Step 6: Straighten and stop
    print("Step 6 - Straightening and stopping...")
    set_steering(0)
    set_speed(0)
    time.sleep(2)
    print("✅ Parked successfully!")

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


#     set_speed(10)
#     print("PARKED")

# def PARKING_SPACE_2():
#     set_speed(10)
#     time.sleep(20)
#     set_steering(-2
#     set_speed(10)
#     time.sleep(5)
#     set_steering(25)
#     set_speed(10)
#     print("PARKED")




