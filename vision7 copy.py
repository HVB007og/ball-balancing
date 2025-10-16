#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math
from IK3 import *
from maskclr import *
vcamno = 0


# # New constants:
# L = 7.94
# L1 = 8.72
# L2 = 4.845
# L3  = 5
# h = 14
# # h = -17
# z = 1

# PID constants
kp = 4e-1
ki = 2e-6
kd = 7e-3

# # Function to get HSV color limits for object detection
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
#     # print(f'Lower HSV Limit: {lower_limit}')
#     # print(f'Upper HSV Limit: {upper_limit}')
#     return lower_limit, upper_limit

d = 5.0
e = 7.937
f = 4.845
g = 8.72
hz = 12.5

ik = IK(d, e, f, g, hz)


# print("here1")
# Main publisher function


a = ik.calculate_a(0, 1)
b = ik.calculate_b(0, 0, 1)
c = ik.calculate_c(0, 0, 1)

print(a, b, c)