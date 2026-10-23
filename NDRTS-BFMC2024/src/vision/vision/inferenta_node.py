# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# import cv2
# from ultralytics import YOLO


# class YoloSubscriber(Node):
#     def __init__(self):
#         super().__init__('yolo8_node')
#         # Parametri pentru model și inferență
#         self.declare_parameter('model_path', 'best.engine')  # poți folosi .engine sau .pt
#         self.declare_parameter('device', 'cuda:0')          # 'cpu' sau 'cuda:0'
#         self.declare_parameter('confidence', 0.80)
#         # Parametri pentru rezoluție (width x height)
#         self.declare_parameter('resolution_width', 640)
#         self.declare_parameter('resolution_height', 480)

#         # Citire parametri
#         model_path = self.get_parameter('model_path').value
#         device     = self.get_parameter('device').value
#         conf       = self.get_parameter('confidence').value
#         width      = self.get_parameter('resolution_width').value
#         height     = self.get_parameter('resolution_height').value

#         # Încarcă modelul YOLO
#         self.model = YOLO(model_path)
#         self.device = device
#         self.confidence = conf
#         self.width = width
#         self.height = height

#         self.bridge = CvBridge()

#         # Subscriber la imaginea raw
#         self.sub = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             10
#         )

#         # Publisher pentru imaginea marcată
#         self.pub = self.create_publisher(
#             Image,
#             '/camera/image_annotated',
#             10
#         )

#         self.get_logger().info(
#             f'Subscribed to /camera/image_raw, publishing /camera/image_annotated. '  
#             f'Resolution set to {self.width}x{self.height}'
#         )

#     def image_callback(self, msg: Image):
#         try:
#             # Conversie ROS Image -> OpenCV BGR
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as e:
#             self.get_logger().error(f'CvBridge Error: {e}')
#             return

#         # Redimensionare dacă parametrul este valid
#         if self.width > 0 and self.height > 0:
#             frame = cv2.resize(frame, (self.width, self.height))

#         # Inferență YOLOv8
#         results = self.model(frame, conf=self.confidence, device=self.device)
#         annotated = results[0].plot()

#         try:
#             # Conversie OpenCV BGR -> ROS Image
#             out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
#             out_msg.header = msg.header
#             self.pub.publish(out_msg)
#         except CvBridgeError as e:
#             self.get_logger().error(f'CvBridge Error: {e}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloSubscriber()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import cv2
from ultralytics import YOLO


class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo8_node')
        # Parametri pentru model și inferență
        self.declare_parameter('model_path', 'best.pt')  # calea către modelul .pt
        self.declare_parameter('device', 'cuda:0')      # 'cpu' sau 'cuda:0'
        self.declare_parameter('confidence', 0.80)
        # Parametri pentru rezoluție (width x height)
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)

        # Citire parametri
        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        conf = self.get_parameter('confidence').value
        width = self.get_parameter('resolution_width').value
        height = self.get_parameter('resolution_height').value

        # Încarcă modelul YOLOv8 (.pt)
        self.model = YOLO(model_path)
        self.device = device
        self.confidence = conf
        self.width = width
        self.height = height

        self.bridge = CvBridge()

        # Subscriber la imaginea raw
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher pentru imaginea marcată (doar regiunea de interes)
        self.pub_image = self.create_publisher(
            Image,
            '/camera/image_annotated',
            10
        )

        # Publisher pentru clasele detectate
        self.pub_class = self.create_publisher(
            String,
            '/detected_class',
            10
        )

        self.get_logger().info(
            f'Subscribed to /camera/image_raw, publishing ROI on /camera/image_annotated and classes on /detected_class. '  
            f'Resolution set to {self.width}x{self.height}, analyzing rightmost quarter.'
        )

    def image_callback(self, msg: Image):
        try:
            # Conversie ROS Image -> OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Redimensionare dacă parametrul este valid
        if self.width > 0 and self.height > 0:
            frame = cv2.resize(frame, (self.width, self.height))

        # Definim regiunea de interes: sfertul drept al imaginii
        h, w = frame.shape[:2]
        x_start = (3 * w) // 4
        roi = frame[:, x_start:]

        # Inferență YOLOv8 doar pe ROI
        results = self.model(roi, conf=self.confidence, device=self.device)
        annotated_roi = results[0].plot()

        # Extragem clasele detectate
        class_indices = results[0].boxes.cls.cpu().numpy().astype(int) if results[0].boxes else []
        class_names = [self.model.names[idx] for idx in class_indices]
        class_msg = String()
        class_msg.data = ','.join(class_names) if class_names else ''
        self.pub_class.publish(class_msg)

        try:
            # Conversie OpenCV BGR -> ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(annotated_roi, encoding='bgr8')
            out_msg.header = msg.header
            self.pub_image.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
