#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
def detect_lines(img, threshold=100, min_line_length=50, max_line_gap=10):
    # Load the image using OpenCV
    # img = cv2.imread(image_path)


    # Convert the image to grayscale
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    # Apply Canny edge detection to identify edges in the image
    edges = cv2.Canny(gray_img, 50, 150, apertureSize=3)


    # Apply Hough Line Transform to detect lines
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold, minLineLength=min_line_length, maxLineGap=max_line_gap)
   
    all_x = []
    # Draw lines on a copy of the original image
    line_img = np.copy(img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
            if np.abs(angle - 90) < 10:  # Check if the line is approximately vertical
                cv2.line(line_img, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw red lines
                all_x.append(x1)
            # cv2.line(line_img, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw red lines


    # Save the result
    return line_img, all_x