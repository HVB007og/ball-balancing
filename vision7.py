#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math
from IK2 import *
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
kp = 0.5
ki = 0 #2e-6
kd = 0 #7e-3

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

# ik = IK(d, e, f, g, hz)
def offfn(theta):
    microstepping = 80/9
    offset = 1/8
    offset2 = 350
    return int(((theta)*microstepping*offset)-offset2)

# print("here1")
# Main publisher function
def Publisher():
    # print("here1")
    orange = [240, 160, 34]  # BGR value for neon orange (this may need adjustment)
    scale = 1
    h, w = int(480*scale), int(640*scale)
    x_origin, y_origin = w // 2, h // 2
    x_origin_real, y_origin_real = 0, 0

    # Initialize video capture
    source = cv2.VideoCapture(vcamno)

    source.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    source.set(cv2.CAP_PROP_FRAME_WIDTH, w)

    # PID control variables
    error_lists = [[], []]
    sum_error = [0, 0]  # For x and y
    diff_error = [0, 0]

    # ROS Publisher and Node
    rospy.init_node('angle_pub_node', anonymous=False)
    pub = rospy.Publisher('angles', String, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

        ret, frame = source.read()
        if not ret:
            # print("here3")
            print("Error: Couldn't read frame.")
            break

        frame = cv2.resize(frame, (w, h))
        frame = cv2.flip(frame, 1)  # Mirror image

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lowerLimit, upperLimit = get_limits(color=orange, hsvImage=hsvImage)

        # Create masks for the target color and flesh tones
        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
        
        # Additional filtering to avoid flesh tones (hue range for flesh tones is roughly 0-20)
        lower_flesh = np.array([0, 20, 70], dtype=np.uint8)  # Typical flesh tones
        upper_flesh = np.array([20, 255, 255], dtype=np.uint8)
        mask_flesh = cv2.inRange(hsvImage, lower_flesh, upper_flesh)

        # Exclude flesh tones from the target color mask
        combined_mask = cv2.bitwise_and(mask, cv2.bitwise_not(mask_flesh))

        # Debugging: Show mask
        cv2.imshow('Mask', combined_mask)

        # Detect contours in the mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check if any contours are detected
        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w2, h2 = cv2.boundingRect(largest_contour)

            # Draw the bounding box on the frame
            cv2.rectangle(frame, (x, y), (x + w2, y + h2), (0, 255, 0), 2)

            # Calculate the center of the bounding box
            x_centre_ball = (x + (x + w2)) // 2
            y_centre_ball = (y + (y + h2)) // 2

            # Calculate real-world coordinates
            x_centre_real = (x_origin - x_centre_ball) / 37.795
            y_centre_real = -(y_origin - y_centre_ball) / 37.795

            # Calculate errors
            x_error = x_origin_real - x_centre_real
            y_error = y_origin_real - y_centre_real

            # PID computations
            sum_error[0] += x_error
            sum_error[1] += y_error
            error_lists[0].append(x_error)
            error_lists[1].append(y_error)

            if len(error_lists[0]) == 2:
                diff_error[0] = (error_lists[0][1] - error_lists[0][0]) / 0.02
                diff_error[1] = (error_lists[1][1] - error_lists[1][0]) / 0.02

            nx = -((kp * x_error) + (ki * sum_error[0]) + (kd * diff_error[0]))
            ny = -((kp * y_error) + (ki * sum_error[1]) + (kd * diff_error[1]))
            nmag = math.sqrt((nx ** 2) + (ny ** 2) + 1)
            

            nx /= nmag
            ny /= nmag
            nz = 1 / nmag

            # Calculate angles

            theta_a = calculate_a(nx, ny, nz)
            theta_b = calculate_b(nx, ny, nz)
            theta_c = calculate_c(nx, ny, nz)
            print(f'{theta_a},{theta_b},{theta_c}')
            # pid bypass
            #theta_a = ik.calculate_a(x_error, y_error, nz)
            #theta_b = ik.calculate_b(x_error, y_error, nz)
            #theta_c = ik.calculate_c(x_error, y_error, nz)

            microstepping = 80/9
            offset = 1/4
            offset2 = 400

            # Publish the calculated angles
            message = f'{offfn(theta_b)},{offfn(theta_a)},{offfn(theta_c)}'
            # message = f'{((theta_a)*microstepping*offset)-offset2:.2f},{((theta_b)*microstepping*offset)-offset2:.2f},{((theta_c)*microstepping*offset)-offset2:.2f}'

            
            pub.publish(message)

            # Display angles on the video frame
            frame = cv2.putText(frame, f'A: {theta_a:.2f} - {nx}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'B: {theta_b:.2f} - {ny}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'C: {theta_c:.2f}', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            #frame = cv2.putText(frame, f'{x_origin} {y_origin}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.circle(frame, (x_centre_ball, y_centre_ball), 10, (0, 0, 255), -1)  # Ball center (red)
            frame = cv2.circle(frame, (x_origin, y_origin), 10, (255, 0, 0), -1)  # Target center (blue)
            frame = cv2.line(frame, (x_centre_ball, y_centre_ball), (x_origin, y_origin), (0, 255, 0), 2)  # Line between ball and target

        else:
            # If no contours are found, publish None
            # print("Bounding Box: None")
            pub.publish('None')

        # Show the processed video frame
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()


        # print("here7")
        # Show the processed video frame
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    source.release()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    try:
        Publisher()
    except rospy.ROSInterruptException:
        pass