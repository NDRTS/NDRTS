import time
import smbus
import rospy
from sensor_msgs.msg import Imu


BNO055_ADDRESS = 0x29
BNO055_REGISTER_EULER_H_LSB = 0x1A
BNO055_REGISTER_ACCEL_DATA_X_LSB = 0x08
DEGREES_PER_LSB = 1.0 / 16.0

def read_bno055_pitch_yaw_roll(bus):
    data = bus.read_i2c_block_data(BNO055_ADDRESS, BNO055_REGISTER_EULER_H_LSB, 6)
    pitch = (data[1] << 8 | data[0])
    roll = (data[3] << 8 | data[2])
    yaw = (data[5] << 8 | data[4])
    if pitch > 32767:
        pitch -= 65536
    if roll > 32767:
        roll -= 65536
    if yaw > 32767:
        yaw -= 65536
    pitch *= DEGREES_PER_LSB
    roll *= DEGREES_PER_LSB
    yaw *= DEGREES_PER_LSB
    return pitch, roll, yaw

def read_bno055_acceleration(bus):
    data = bus.read_i2c_block_data(BNO055_ADDRESS, BNO055_REGISTER_ACCEL_DATA_X_LSB, 6)
    accel_x = (data[1] << 8 | data[0])
    accel_y = (data[3] << 8 | data[2])
    accel_z = (data[5] << 8 | data[4])
    if accel_x > 32767:
        accel_x -= 65536
    if accel_y > 32767:
        accel_y -= 65536
    if accel_z > 32767:
        accel_z -= 65536
    accel_x /= 100.0
    accel_y /= 100.0
    accel_z /= 100.0
    return accel_x, accel_y, accel_z

def main():
    bus = smbus.SMBus(1)
    rospy.init_node('imu_publisher', anonymous=True)
    imu_publisher = rospy.Publisher('/imu', Imu, queue_size=10)

    try:
        while not rospy.is_shutdown():
            pitch, roll, yaw = read_bno055_pitch_yaw_roll(bus)
            accel_x, accel_y, accel_z = read_bno055_acceleration(bus)

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.orientation.x = roll
            imu_msg.orientation.y = pitch
            imu_msg.orientation.z = yaw
            imu_msg.angular_velocity.x = 0.0  # Assuming no gyroscope data
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            imu_publisher.publish(imu_msg)
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
