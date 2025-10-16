#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math
from IK2 import *



def get_limits(color, hsvImage):
    # Convert the color from BGR to HSV
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    hue = hsvC[0][0][0]
    
    # Define HSV ranges to include neon orange and yellowish colors
    if hue >= 165:  # Wrap around the hue range
        lower_limit = np.array([0, 100, 150], dtype=np.uint8)  # Lowered the min saturation
        upper_limit = np.array([30, 255, 255], dtype=np.uint8)  # Extended hue range to include yellowish tones
    elif hue <= 15:
        lower_limit = np.array([0, 100, 150], dtype=np.uint8)  # Lowered the min saturation
        upper_limit = np.array([30, 255, 255], dtype=np.uint8)  # Extended hue range to include yellowish tones
    else:
        lower_limit = np.array([10, 100, 150], dtype=np.uint8)  # Lowered the min saturation
        upper_limit = np.array([30, 255, 255], dtype=np.uint8)  # Extended hue range to include yellowish tones
    
    # print(f'Lower HSV Limit: {lower_limit}')
    # print(f'Upper HSV Limit: {upper_limit}')
    return lower_limit, upper_limit

# def get_limits(color, hsvImage):
#     # Convert the color from BGR to HSV
#     c = np.uint8([[color]])
#     hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
#     hue = hsvC[0][0][0]
    
#     # Define HSV ranges to include neon orange and yellowish colors, while excluding flesh tones
#     if hue >= 165 or hue <= 15:  # Neon orange and some yellowish tones
#         lower_limit = np.array([0, 100, 150], dtype=np.uint8)  # Lowered the min saturation
#         upper_limit = np.array([30, 255, 255], dtype=np.uint8)  # Extended hue range to include yellowish tones
#     else:
#         lower_limit = np.array([10, 100, 150], dtype=np.uint8)  # Adjusted lower bound for non-flesh tones
#         upper_limit = np.array([30, 255, 255], dtype=np.uint8)  # Extended hue range to include yellowish tones
    
#     # Print HSV limits for debugging
#     print(f'Lower HSV Limit: {lower_limit}')
#     print(f'Upper HSV Limit: {upper_limit}')
#     return lower_limit, upper_limit
