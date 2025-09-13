#!/usr/bin/env python3
import rospy
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from torchvision import transforms

# Initialize node
rospy.init_node('detection_node', anonymous=True)

# Publisher for detection results
result_pub = rospy.Publisher('/results', String, queue_size=10)

# Create CvBridge
bridge = CvBridge()

# Load model
model = torch.load('best.pt')
model.eval()

# Preprocessing pipeline
preprocess = transforms.Compose([
    transforms.ToTensor(),
    transforms.Resize((224, 224)),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

def image_callback(msg):
    try:
        # Convert ROS Image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Preprocess the image
        input_tensor = preprocess(cv_image).unsqueeze(0)

        # Perform inference
        with torch.no_grad():
            output = model(input_tensor)

        # Parse the output (customize as per your model's output format)
        result = f"Detected class: {output.argmax(1).item()}"

        # Publish the result
        result_pub.publish(result)

        # Display the image
        cv2.imshow("Detection", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(f"Error in processing image: {e}")

# Subscribe to camera topic
rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

rospy.spin()

