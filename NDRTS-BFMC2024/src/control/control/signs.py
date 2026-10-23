import time
import serial
from std_msgs.msg import Int32
from control.engines.messageconverter import MessageConverter


class TrafficSignManeuvers:
    def __init__(self, node):
        self.node = node
        self.dist_front = 9999.0
        self.line = 0

        self.speed_pub = node.create_publisher(Int32, '/speed', 10)
        self.dir_pub = node.create_publisher(Int32, '/direction_lane', 10)
        self.stop_lanekeeping_pub = node.create_publisher(
            Int32, '/stop_lanekeeping_cmd', 10)

        self.serial = serial.Serial("/dev/ttyACM0", 115200)
        self.mc = MessageConverter()

    def send_speed(self, speed: int):
        self.speed_pub.publish(Int32(data=speed))

    def send_steering(self, angle: int):
        self.dir_pub.publish(Int32(data=angle*10))

    def _serial_speed(self, cms: int):
        cmd = self.mc.get_command("speed", speed=int(cms))
        self.serial.write(cmd.encode())

    def _serial_steer(self, deg: int):
        cmd = self.mc.get_command("steer", steerAngle=int(deg))
        self.serial.write(cmd.encode())

    def send_stop_signal(self):
        self.stop_lanekeeping_pub.publish(Int32(data=1))

    def send_resume_signal(self):
        self.stop_lanekeeping_pub.publish(Int32(data=0))

    def handle_maneuver(self, label: str):
        if label == "Stopsign":
            self.send_speed(0)
            time.sleep(10)
            self.send_speed(10)

        elif label == "Crosswalksign":
            start = time.time()
            while time.time() - start < 10:
                if self.dist_front <= 30:
                    self.send_speed(0)
                else:
                    self.send_speed(10)
                time.sleep(0.25)

        elif label == "Prioritysign":
            self.send_speed(10)

        elif label == "Round-aboutsign" and self.line == 1:
            self.send_stop_signal()
            time.sleep(1.5)
            self.roundabout()
            self.send_resume_signal()

        elif label == "Parkingsign":
            self.send_stop_signal()
            time.sleep(1.5)
            self.parallel_park_left()

        else:
            self.node.get_logger().info(f"⚠️ No handler defined for: {label}")

    def roundabout(self):
        self._serial_speed(10)
        self._serial_steer(0)
        time.sleep(1.5)

        self._serial_steer(25)
        time.sleep(4.5)

        self._serial_steer(-15)
        time.sleep(1)

        self._serial_steer(-25)
        time.sleep(19)

        self._serial_steer(25)
        time.sleep(3)

    def parallel_park_left(self):
        self.node.get_logger().info("🅿️ Initiating LEFT parallel parking maneuver")
        self.send_speed(0)
        self.send_steering(0)
        time.sleep(1)
        self.send_speed(-5)
        self.send_steering(-20)
        time.sleep(15)
        self.send_steering(0)
        time.sleep(3)
        self.send_steering(20)
        time.sleep(10)
        self.send_steering(-20)
        self.send_speed(5)
        time.sleep(5)
        self.send_steering(0)
        self.send_speed(0)
        time.sleep(2)
        self.node.get_logger().info("✅ Parked successfully!")
