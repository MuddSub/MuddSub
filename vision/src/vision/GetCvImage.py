#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2
from std_msgs.msg import Header, Float32
from vision.msg import BuoyPos
import os
import sys

bridge = CvBridge()

def publishBuoyPos(center_x, center_y):
    """
    Publish a drivers/Depth message.

    Uses the _depthPub Publisher to publish to drivers/depth_sensor/depth.

    Args:
            depthStamped: The drivers/Depth message.
    """
    centerMsg = BuoyPos()
    centerMsg.header.frame_id = "FrontCam/BuoyPos"
    centerMsg.header.stamp = rospy.Time.now()
    centerMsg.center_x = center_x
    centerMsg.center_y = center_y
    pub.publish(centerMsg)
                
if __name__ == '__main__':
    rospy.loginfo("SaveImages is launched")
    rospy.init_node('vision_subscriber')
    # sys.argv[1]
    # rospy.Subscriber(f"/{sys.argv[1]}/image_raw", Image, callback)
    pub = rospy.Publisher("/buoy_pos", Float32, queue_size=1) # Publishes Buoy Position relative to center of screen -1 to 1
    rate = rospy.Rate(10) #not necessary?

    def callback(image):
        
        
        
        while not rospy.is_shutdown():
            frame = bridge.imgmsg_to_cv2(image, "bgr8")

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

            lower_red1 = np.array([0, 120, 70]) 
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1) 
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            contours,  = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            cx, cy = 0, 0


            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            result = cv2.bitwise_and(frame, frame, mask = mask) 

            cv2.imshow('frame', frame)
            cv2.imshow('mask', mask)
            cv2.imshow('result', result)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # cv2.imwrite("file.jpg", frame)
            # # It converts the BGR color space of image to HSV color space 
            # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
            
            # # Threshold of blue in HSV space  # ADJUST FILTER RANGES
            # lower_red = np.array([0, 35, 140]) 
            # upper_red = np.array([55, 255, 255]) 
        
            # # preparing the mask to overlay 
            # mask = cv2.inRange(hsv, lower_red, upper_red) 
            
            # # The black region in the mask has the value of 0, 
            # # so when multiplied with original image removes all non-blue regions 
            # result = cv2.bitwise_and(frame, frame, mask = mask) 
        

            # print("Current working directory: {0}".format(os.getcwd()))
            # now = rospy.get_rostime() # current date and time
        
            
            
            # rospy.loginfo("hello")
            publishBuoyPos(cx, cy)
            # pub.publish(cx, cy)
            rate.sleep()

        # image_name = f"{now.secs},{now.nsecs}.jpg"
        # print(f"The image name is {image_name}")

        # cv2.imwrite(image_name,image_message)
        # input_img = np.copy(image_message).astype(float)
        rospy.sleep(5)
        
    rospy.Subscriber("/usb_cam_0/image_raw", Image, callback)
    
    
    rospy.spin()