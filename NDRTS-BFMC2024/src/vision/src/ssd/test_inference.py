import torch
import torchvision.transforms as transforms
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Float32, String
from cnn_nvidia import Nvidia_Model

# Assuming your Nvidia_Model class is defined as in your provided code

# Function to load a pretrained model and move it to GPU
def load_model(model, model_path):
    try:
        model.load_state_dict(torch.load(model_path))
        print(f"Model loaded successfully from '{model_path}'")
        model.eval()
        model.cuda()  # Move model to GPU
        return model
    except Exception as e:
        print(f"Failed to load model from '{model_path}': {e}")
        return None

# Function to preprocess and predict using the loaded model
def predict_frame(model, frame, transform, gps_coords):
    # Preprocess the frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
    frame = cv2.resize(frame, (320, 160))  # Resize to model's input size
    frame_tensor = transform(frame).unsqueeze(0)  # Apply transforms and add batch dimension
    
    # Move frame tensor to GPU
    frame_tensor = frame_tensor.cuda()

    # Perform prediction
    with torch.no_grad():
        output = model(frame_tensor)
    
    # Move output tensor back to CPU and convert to numpy array
    output = output.cpu().numpy()

    return output

class RosInferenceNode:
    def __init__(self, model_path):
        self.bridge = CvBridge()
        self.model = Nvidia_Model()
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # Example normalization values
        ])
        
        self.model = load_model(self.model, model_path)
        
        self.direction_pub = rospy.Publisher('/direction_lane', Float32, queue_size=10)
        self.speed_pub = rospy.Publisher('/speed', Float32, queue_size=10)
        
        self.gps_coords = [0.0, 0.0]  # Initialize GPS coordinates
        self.speed_ema = None
        self.direction_ema = None
        self.alpha = 0.2  # EMA smoothing factor

        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/gps_data', String, self.gps_callback)

    def gps_callback(self, msg):
        try:
            coords = msg.data.split(',')
            self.gps_coords = [float(coords[0]), float(coords[1])]
        except Exception as e:
            rospy.logerr(f"Failed to process GPS data: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            output = predict_frame(self.model, cv_image, self.transform, self.gps_coords)
            
            direction = output[0][0]  # Assuming the first output is direction
            speed = output[0][1]  # Assuming the second output is speed
            
            # Apply exponential moving average (EMA) to smooth values
            if self.direction_ema is None:
                self.direction_ema = direction
            else:
                self.direction_ema = self.alpha * direction + (1 - self.alpha) * self.direction_ema
            
            if self.speed_ema is None:
                self.speed_ema = speed
            else:
                self.speed_ema = self.alpha * speed + (1 - self.alpha) * self.speed_ema
            
            self.direction_pub.publish(Float32(self.direction_ema))
            self.speed_pub.publish(Float32(self.speed_ema))
        
        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

if __name__ == "__main__":
    import argparse

    # Argument parsing
    parser = argparse.ArgumentParser(description='Real-time Inference with Nvidia Model on ROS')
    parser.add_argument('--model_path', type=str, help='Path to the trained model file')
    args = parser.parse_args()

    # Check if model_path is provided
    if not args.model_path:
        print("Please provide --model_path argument.")
    else:
        rospy.init_node('ros_inference_node', anonymous=True)
        ros_inference_node = RosInferenceNode(args.model_path)
        
        rospy.spin()
