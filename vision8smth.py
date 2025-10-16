import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math
from IK3 import *
from maskclr import *

vcamno = 0

# PID constants
kp = 12
ki = 0  # 2e-6
kd = 0  # 7e-3

d = 5.0
e = 7.937
f = 4.845
g = 8.72
hz = 12.5

# ik = IK(d, e, f, g, hz)

def Publisher():
    orange = [240, 160, 34]  # BGR value for neon orange (this may need adjustment)
    h, w = 480, 640
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
            print("Error: Couldn't read frame.")
            break

        frame = cv2.resize(frame, (w, h))
        frame = cv2.flip(frame, 1)  # Mirror image

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lowerLimit, upperLimit = get_limits(color=orange, hsvImage=hsvImage)

        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
        
        # Additional filtering to avoid flesh tones (hue range for flesh tones is roughly 0-20)
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
                diff_error[0] = (error_lists[0][1] - error_lists[0][0]) / 0.02
                diff_error[1] = (error_lists[1][1] - error_lists[1][0]) / 0.02

            # PID control to calculate the n vector
            nx = -(kp * x_error + ki * sum_error[0] + kd * diff_error[0])
            ny = -(kp * y_error + ki * sum_error[1] + kd * diff_error[1])
            nmag = math.sqrt((nx ** 2) + (ny ** 2) + 1)

            nx /= nmag
            ny /= nmag
            nz = 1 / nmag

            # Visualize the `n` vector as an arrow
            # arrow_length = math.sqrt(nx**2+ny**2+1)*100 # Length of the arrow for visualization
            arrow_length = math.sqrt(x_error**2+y_error**2)*10 # Length of the arrow for visualization
            arrow_tip_x = int(x_centre_ball + nx * arrow_length)
            arrow_tip_y = int(y_centre_ball - ny * arrow_length)

            # Draw the `n` vector as an arrow
            cv2.arrowedLine(frame, (x_centre_ball, y_centre_ball), (arrow_tip_x, arrow_tip_y), (0, 0, 0), 2)

            # # Calculate angles
            # theta_a = ik.calculate_a(ny, nz)
            # theta_b = ik.calculate_b(nx, ny, nz)
            # theta_c = ik.calculate_c(nx, ny, nz)

            theta_a = calculate_a(nx, ny, nz)
            theta_b = calculate_b(nx, ny, nz)
            theta_c = calculate_c(nx, ny, nz)
            microstepping = 80 / 9
            offset = 1 / 4
            offset2 = 400

            message = f'{int(((theta_a) * microstepping * offset) - offset2)},{int(((theta_b) * microstepping * offset) - offset2)},{int(((theta_c) * microstepping * offset) - offset2)}'
            pub.publish(message)

            # Display angles on the video frame
            frame = cv2.putText(frame, f'A: {theta_a:.2f} - {nx:2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'B: {theta_b:.2f} - {ny:2f}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'C: {theta_c:.2f}', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            # Plotting center of the mask (ball) and target center
            cv2.circle(frame, (x_centre_ball, y_centre_ball), 10, (0, 0, 255), -1)  # Ball center (red)
            cv2.circle(frame, (x_origin, y_origin), 10, (255, 0, 0), -1)  # Target center (blue)
            cv2.line(frame, (x_centre_ball, y_centre_ball), (x_origin, y_origin), (0, 255, 0), 2)  # Line between ball and target

        else:
            pub.publish('None')

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
