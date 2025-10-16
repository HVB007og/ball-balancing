#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math
vcamno = 1


# New constants:
L = 7.94
L1 = 8.72
L2 = 4.845
L3  = 5
h = 14
# h = -17
z = 1

# PID constants
kp = 4e-4
ki = 2e-5
kd = 7e-3

# Function to calculate theta_a
def calculate_a(nx, ny, nz):
    print(f'nx:{nx},ny:{ny},nz:{nz}')
    # Adjust for negative square root argument if encountered
    sqrt_argument_a = L**2 - (h - z)**2
    if sqrt_argument_a < 0:
        print(f"Inverting sqrt_argument_a sign: {sqrt_argument_a}")
        sqrt_argument_a = abs(sqrt_argument_a)
    
    a = math.sqrt(sqrt_argument_a)
    b = 0
    c = h - ((nx * L) / math.sqrt(nx**2 + nz**2))

    # Intermediate simplifications
    A = (a - L3) / c
    B = (a**2 + c**2 + L2**2 - L1**2 - L3**2) / (2 * c)
    D = A**2 + 1
    E = 2 * (A * B + L3)
    F = B**2 - L2**2 - L3**2

    # Adjust for negative discriminant if encountered
    discriminant = E**2 - 4 * D * F
    if discriminant < 0:
        print(f"Inverting discriminant sign: {discriminant}")
        discriminant = abs(discriminant)
    
    d = (-E + math.sqrt(discriminant)) / (2 * D)

    # Check for negative square root argument in the calculation of f
    sqrt_argument_f = L2**2 - d**2 + 2 * L1 * d - L1**2
    if sqrt_argument_f < 0:
        print(f"Skipping this iteration due to invalid sqrt_argument_f: {sqrt_argument_f}")
        sqrt_argument_f = abs(sqrt_argument_f)
        # return 100  # Skip this iteration
    
    f = math.sqrt(sqrt_argument_f)
    a_theta = 90 - math.degrees(math.atan2(f / L2, (d - L3) / L2))

    return a_theta


# Function to calculate theta_b
def calculate_b(nx, ny, nz):
    b_theta = 0 # temp
    # bx = (math.sqrt(3) / 2) * (e * (1 - (nx ** 2 + math.sqrt(3) * nx * ny) / (nz + 1)) - d)
    # by = bx / math.sqrt(3)
    # bz = hz - ((e / 2) * (math.sqrt(3) * nx + ny))
    # bm = math.sqrt(bx ** 2 + by ** 2 + bz ** 2)

    # acos_input_1 = min(1, max(-1, (math.sqrt(3) * bx + by) / (-2 * bm)))
    # acos_input_2 = min(1, max(-1, (bm ** 2 + f ** 2 - g ** 2) / (2 * bm * f)))

    # b_theta = math.acos(acos_input_1) + math.acos(acos_input_2)
    return b_theta

# Function to calculate theta_c
def calculate_c(nx, ny, nz):
    c_theta = 0 # temp
    # cx = (math.sqrt(3) / 2) * (d - e * (1 - (nx ** 2 - math.sqrt(3) * nx * ny) / (nz + 1)))
    # cy = -cx / math.sqrt(3)
    # cz = hz + (e / 2) * (math.sqrt(3) * nx - ny)
    # cm = math.sqrt(cx ** 2 + cy ** 2 + cz ** 2)

    # acos_input_1 = min(1, max(-1, (math.sqrt(3) * cx - cy) / (2 * cm)))
    # acos_input_2 = min(1, max(-1, (cm ** 2 + f ** 2 - g ** 2) / (2 * cm * f)))

    # c_theta = math.acos(acos_input_1) + math.acos(acos_input_2)
    return c_theta

# Function to get HSV color limits for object detection
def get_limits(color, hsvImage):
    # Convert the color from BGR to HSV
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    hue = hsvC[0][0][0]
    
    # Define HSV ranges to include neon orange and yellowish colors, while excluding flesh tones
    if hue >= 165 or hue <= 15:  # Neon orange and some yellowish tones
        lower_limit = np.array([0, 100, 150], dtype=np.uint8)  # Lowered the min saturation
        upper_limit = np.array([30, 255, 255], dtype=np.uint8)  # Extended hue range to include yellowish tones
    else:
        lower_limit = np.array([10, 100, 150], dtype=np.uint8)  # Adjusted lower bound for non-flesh tones
        upper_limit = np.array([30, 255, 255], dtype=np.uint8)  # Extended hue range to include yellowish tones
    
    # Print HSV limits for debugging
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
    source = cv2.VideoCapture(vcamno)

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
            x_centre_ball = x + w2 // 2
            y_centre_ball = y + h2 // 2

            # Calculate real-world coordinates
            x_centre_real = (x_origin - x_centre_ball)
            y_centre_real = -(y_origin - y_centre_ball)

            # Calculate errors
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
            theta_a = calculate_a(nx, ny, nz)
            theta_b = calculate_b(nx, ny, nz)
            theta_c = calculate_c(nx, ny, nz)

            # Publish the calculated angles
            message = f'{theta_a:.2f}, {theta_b:.2f}, {theta_c:.2f}'
            pub.publish(message)

            # Display angles on the video frame
            frame = cv2.putText(frame, f'A: {theta_a:.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'B: {theta_b:.2f}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, f'C: {theta_c:.2f}', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        else:
            # If no contours are found, publish None
            print("Bounding Box: None")
            pub.publish('None')

        # Show the processed video frame
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()



        
        
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