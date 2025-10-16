#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math

# Constants for your mechanical system
d = 5.0
e = 7.937
f = 4.845
g = 8.72
hz = 12.5

# PID constants
kp = 4e-4
ki = 2e-6
kd = 7e-3

# Function to calculate theta_a
def calculate_a(nx, ny, nz):
    ay = d + ((e / 2) * (1 - ((nx ** 2 + 3 * nz ** 2 + 3 * nz) / (nz + 1 - nx ** 2))))
    az = hz + (e * ny)
    am = math.sqrt(ay ** 2 + az ** 2)

    # Safe check to ensure acos input is within [-1, 1]
    acos_input_1 = min(1, max(-1, ay / am))
    acos_input_2 = min(1, max(-1, (am ** 2 + f ** 2 - g ** 2) / (2 * am * f)))

    a_theta = math.acos(acos_input_1) + math.acos(acos_input_2)
    return a_theta

# Function to calculate theta_b
def calculate_b(nx, ny, nz):
    bx = (math.sqrt(3) / 2) * (e * (1 - (nx ** 2 + math.sqrt(3) * nx * ny) / (nz + 1)) - d)
    by = bx / math.sqrt(3)
    bz = hz - ((e / 2) * (math.sqrt(3) * nx + ny))
    bm = math.sqrt(bx ** 2 + by ** 2 + bz ** 2)

    acos_input_1 = min(1, max(-1, (math.sqrt(3) * bx + by) / (-2 * bm)))
    acos_input_2 = min(1, max(-1, (bm ** 2 + f ** 2 - g ** 2) / (2 * bm * f)))

    b_theta = math.acos(acos_input_1) + math.acos(acos_input_2)
    return b_theta

# Function to calculate theta_c
def calculate_c(nx, ny, nz):
    cx = (math.sqrt(3) / 2) * (d - e * (1 - (nx ** 2 - math.sqrt(3) * nx * ny) / (nz + 1)))
    cy = -cx / math.sqrt(3)
    cz = hz + (e / 2) * (math.sqrt(3) * nx - ny)
    cm = math.sqrt(cx ** 2 + cy ** 2 + cz ** 2)

    acos_input_1 = min(1, max(-1, (math.sqrt(3) * cx - cy) / (2 * cm)))
    acos_input_2 = min(1, max(-1, (cm ** 2 + f ** 2 - g ** 2) / (2 * cm * f)))

    c_theta = math.acos(acos_input_1) + math.acos(acos_input_2)
    return c_theta

# Function to get HSV color limits for object detection
def get_limits(color):
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
    
    print(f'Lower HSV Limit: {lower_limit}')
    print(f'Upper HSV Limit: {upper_limit}')
    return lower_limit, upper_limit




print("here1")
# Main publisher function
def Publisher():
    print("here1")
    orange = [240, 160, 34]  # BGR value for neon orange (this may need adjustment)
    h, w = 600, 600
    x_origin, y_origin = w // 2, h // 2
    x_origin_real, y_origin_real = 0, 0

    # Initialize video capture
    source = cv2.VideoCapture(4)
    source.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    source.set(cv2.CAP_PROP_FRAME_WIDTH, w)

    # PID control variables
    error_lists = [[], []]
    sum_error = [0, 0]  # For x and y

    # ROS Publisher and Node
    rospy.init_node('angle_pub_node', anonymous=False)
    pub = rospy.Publisher('angles', String, queue_size=10)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        print("here2")
        ret, frame = source.read()
        if not ret:
            print("here3")
            print("Error: Couldn't read frame.")
            break

        frame = cv2.resize(frame, (w, h))
        frame = cv2.flip(frame, 1)  # Mirror image

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lowerLimit, upperLimit = get_limits(color=orange)

        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

        # Debugging: Show mask
        cv2.imshow('Mask', mask)

        mask_ = Image.fromarray(mask)
        bbox = mask_.getbbox()
        print("here4")
        if bbox is not None:
            print("here5")
            x1, y1, x2, y2 = bbox
            x_centre_ball = (x1 + x2) // 2
            y_centre_ball = (y1 + y2) // 2

            x_centre_real = (x_origin - x_centre_ball)
            y_centre_real = -(y_origin - y_centre_ball)

            x_error = x_origin_real - x_centre_real
            y_error = y_origin_real - y_centre_real

            # PID computations
            sum_error[0] += x_error
            sum_error[1] += y_error
            error_lists[0].append(x_error)
            error_lists[1].append(y_error)

            if len(error_lists[0]) == 2:
                diff_error_x = (error_lists[0][1] - error_lists[0][0]) / 0.02
                diff_error_y = (error_lists[1][1] - error_lists[1][0]) / 0.02
            else:
                diff_error_x = 0
                diff_error_y = 0

            nx = -(kp * x_error + ki * sum_error[0] + kd * diff_error_x)
            ny = -(kp * y_error + ki * sum_error[1] + kd * diff_error_y)
            nmag = math.sqrt(nx ** 2 + ny ** 2 + 1)

            nx /= nmag
            ny /= nmag
            nz = 1 / nmag

            # Calculate angles
            theta_a = calculate_a(nx, ny, nz) * (180 / math.pi)
            theta_b = calculate_b(nx, ny, nz) * (180 / math.pi)
            theta_c = calculate_c(nx, ny, nz) * (180 / math.pi)

            message = f'{theta_a:.2f}, {theta_b:.2f}, {theta_c:.2f}'
            pub.publish(message)

            # Display angles on the video frame
            frame = cv2.putText(frame, f'A: {theta_a:.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'B: {theta_b:.2f}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'C: {theta_c:.2f}', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        else:
            print("here6")
            print("Bounding Box: None")
            pub.publish('None')
        print("here7")
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