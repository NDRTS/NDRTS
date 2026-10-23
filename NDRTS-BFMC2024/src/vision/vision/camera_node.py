# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge

# import cv2
# import base64


# class CameraPublisher(Node):
#     def __init__(self):
#         super().__init__('camera_node')

#         self.bridge = CvBridge()

#         self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
#         self.base64_pub = self.create_publisher(
#             String, '/raw_image_base64', 10)

#         # GStreamer pipeline
#         self.gst_pipeline = (
#             "v4l2src device=/dev/video0 ! "
#             "video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! "
#             "videoconvert ! video/x-raw, format=BGR ! "
#             "appsink drop=true sync=false"
#         )

#         self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)

#         # self.cap = cv2.VideoCapture(0)  # No GStreamer, just use default

#         if not self.cap.isOpened():
#             self.get_logger().error("Camera cannot be opened.")
#             return

#         self.get_logger().info(
#             "Camera node started. Publishing to /camera/image_raw and /raw_image_base64")

#         timer_period = 0.1  # 10Hz
#         self.timer = self.create_timer(timer_period, self.publish_frame)

#     def publish_frame(self):
#         ret, frame = self.cap.read()
#         if not ret:
#             self.get_logger().error("Error reading frame from camera.")
#             return

#         # GPU processing
#         gpu_frame = cv2.cuda_GpuMat()
#         gpu_frame.upload(frame)
#         gpu_gray = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)
#         processed_frame = gpu_gray.download()

#         # Adjust brightness
#         adjusted_frame = cv2.convertScaleAbs(
#             processed_frame, alpha=0.9, beta=0)

#         # Publish grayscale image
#         msg = self.bridge.cv2_to_imgmsg(adjusted_frame, encoding='mono8')
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "camera_frame"
#         self.image_pub.publish(msg)

#         # Publish base64 string of original frame
#         _, jpeg_img = cv2.imencode('.jpg', frame)
#         base64_str = base64.b64encode(jpeg_img).decode('utf-8')
#         self.base64_pub.publish(String(data=base64_str))

#     def destroy_node(self):
#         self.cap.release()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()



#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import base64


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.base64_pub = self.create_publisher(String, '/raw_image_base64', 10)

        # GStreamer pipeline
        self.gst_pipeline = (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )

        self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)
        # alternativ fără GStreamer:
        # self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Camera cannot be opened.")
            return

        self.get_logger().info(
            "Camera node started. Publishing to /camera/image_raw and /raw_image_base64"
        )

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Error reading frame from camera.")
            return

        # --- Converitrea în grayscale a fost dezactivată ---
        # GPU processing (grayscale) – comentat:
        # gpu_frame = cv2.cuda_GpuMat()
        # gpu_frame.upload(frame)
        # gpu_gray = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)
        # processed_frame = gpu_gray.download()
        #
        # # Ajustare luminozitate pe grayscale
        # adjusted_frame = cv2.convertScaleAbs(
        #     processed_frame, alpha=0.9, beta=0)
        #
        # # Publish grayscale image
        # msg = self.bridge.cv2_to_imgmsg(adjusted_frame, encoding='mono8')

        # Publish original BGR image color
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self.image_pub.publish(msg)

        # Publish base64 string of original frame
        _, jpeg_img = cv2.imencode('.jpg', frame)
        base64_str = base64.b64encode(jpeg_img).decode('utf-8')
        self.base64_pub.publish(String(data=base64_str))

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
