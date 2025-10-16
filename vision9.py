#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math
from IK2 import *
from maskclr import *
import os  # To check if the file exists

vcamno = 4

# Default PID constants
kp = 0.5
ki = 0
kd = 0

pid_file = "/home/hvb/catkin_ws/pid_values.txt"  # File path where PID values will be read from

d = 5.0
e = 7.937
f = 4.845
g = 8.72
hz = 12.5

def offfn(theta):
    microstepping = 80/9
    offset = 1/4
    offset2 = 350
    return int(((theta)*microstepping*offset)-offset2)

# Function to read PID values from the file
def read_pid_values():
    global kp, ki, kd
    if os.path.exists(pid_file):
        with open(pid_file, 'r') as file:
            pid_values = file.read().strip().split(',')
            if len(pid_values) == 3:
                try:
                    kp = float(pid_values[0])
                    ki = float(pid_values[1])
                    kd = float(pid_values[2])
                except ValueError:
                    print("Error: Invalid PID values in the file.")
    else:
        print("PID values file not found, using default values.")

# Main publisher function
def Publisher():
    orange = [240, 160, 34]
    scale = 1
    h, w = int(480*scale), int(640*scale)
    x_origin, y_origin = w // 2, h // 2
    x_origin_real, y_origin_real = 0, 0

    source = cv2.VideoCapture(vcamno)
    source.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    source.set(cv2.CAP_PROP_FRAME_WIDTH, w)

    error_lists = [[], []]
    sum_error = [0, 0]
    diff_error = [0, 0]

    rospy.init_node('angle_pub_node', anonymous=False)
    pub = rospy.Publisher('angles', String, queue_size=10)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        # Read updated PID values
        read_pid_values()

        ret, frame = source.read()
        if not ret:
            print("Error: Couldn't read frame.")
            break

        frame = cv2.resize(frame, (w, h))
        frame = cv2.flip(frame, 1)

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lowerLimit, upperLimit = get_limits(color=orange, hsvImage=hsvImage)

        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

        lower_flesh = np.array([0, 20, 70], dtype=np.uint8)
        upper_flesh = np.array([20, 255, 255], dtype=np.uint8)
        mask_flesh = cv2.inRange(hsvImage, lower_flesh, upper_flesh)

        combined_mask = cv2.bitwise_and(mask, cv2.bitwise_not(mask_flesh))

        cv2.imshow('Mask', combined_mask)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w2, h2 = cv2.boundingRect(largest_contour)

            cv2.rectangle(frame, (x, y), (x + w2, y + h2), (0, 255, 0), 2)

            x_centre_ball = (x + (x + w2)) // 2
            y_centre_ball = (y + (y + h2)) // 2

            x_centre_real = (x_origin - x_centre_ball) / 37.795
            y_centre_real = -(y_origin - y_centre_ball) / 37.795

            x_error = x_origin_real - x_centre_real
            y_error = y_origin_real - y_centre_real

            sum_error[0] += x_error
            sum_error[1] += y_error
            error_lists[0].append(x_error)
            error_lists[1].append(y_error)

            if len(error_lists[0]) == 2:
                diff_error[0] = (error_lists[0][1] - error_lists[0][0]) / 0.01
                diff_error[1] = (error_lists[1][1] - error_lists[1][0]) / 0.01

            nx = -((kp * x_error) + (ki * sum_error[0]) + (kd * diff_error[0]))
            ny = -((kp * y_error) + (ki * sum_error[1]) + (kd * diff_error[1]))
            nmag = math.sqrt((nx ** 2) + (ny ** 2) + 1)

            nx /= nmag
            ny /= nmag
            nz = 1 / nmag

            theta_a = calculate_a(nx, ny, nz)
            theta_b = calculate_b(nx, ny, nz)
            theta_c = calculate_c(nx, ny, nz)
            print(f'{theta_a},{theta_b},{theta_c}')

            message = f'{offfn(theta_b)},{offfn(theta_a)},{offfn(theta_c)}'
            pub.publish(message)

            frame = cv2.putText(frame, f'A: {theta_a:.2f} - {nx}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'B: {theta_b:.2f} - {ny}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'C: {theta_c:.2f}', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.circle(frame, (x_centre_ball, y_centre_ball), 10, (0, 0, 255), -1)
            frame = cv2.circle(frame, (x_origin, y_origin), 10, (255, 0, 0), -1)
            frame = cv2.line(frame, (x_centre_ball, y_centre_ball), (x_origin, y_origin), (0, 255, 0), 2)

        else:
            pub.publish('None')

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
