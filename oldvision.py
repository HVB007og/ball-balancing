#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math

yellow = [0, 255, 255]
h = 480
w = 640
y_origin = h//2
x_origin = w//2
x_origin_real = 0
y_origin_real = 0
source = cv2.VideoCapture(0)
source.set(cv2.CAP_PROP_FRAME_HEIGHT,h)
source.set(cv2.CAP_PROP_FRAME_WIDTH,w)

d = 5.0
e = 7.937
f = 4.845
g = 8.72
h = 12.5

#camera_matrix = np.load('../../../calibrationFiles/cameraMatrix.npy')
#dist_coefs = np.load('../../../calibrationFiles/cameraDistortion.npy')

'''def calibrate(image, camera_matrix, dist_coefs):
    K_undistort = camera_matrix

    img_und = cv2.undistort(image, camera_matrix, dist_coefs, newCameraMatrix=K_undistort)

    return(img_und)
'''

def calculate_a(nx, ny, nz):
    ay = d+((e/2)*(1 - ((math.pow(nx, 2) + (3*math.pow(nz, 2)) + (3*nz))/(nz + 1 - math.pow(nx, 2))) + ((math.pow(nx,4) - (3*math.pow(nx,2)*math.pow(ny,2)))/((nz+1)*(nz+1-math.pow(nx,2))))))
    az = h + (e*ny)
    am = math.sqrt(math.pow(ay,2) + math.pow(az,2))
    a_theta = math.acos(ay/am) + math.acos((math.pow(am,2)+math.pow(f,2)-math.pow(g,2))/(2*am*f))
    return(a_theta)

def calculate_b(nx, ny, nz):
    bx = (math.sqrt(3)/2) * ((e * (1 - ((math.pow(nx, 2) + (math.sqrt(3)*nx*ny))/(nz + 1)))) - d)
    by = bx/math.sqrt(3)
    bz = h - ((e/2)*((math.sqrt(3)*nx) + ny))
    bm = math.sqrt(math.pow(bx, 2) + math.pow(by, 2) + math.pow(bz, 2))
    b_theta = math.acos(((math.sqrt(3)*bx) + by)/(-2*bm)) + math.acos((math.pow(bm, 2) + math.pow(f, 2) - math.pow(g,2))/(2*bm*f))
    return(b_theta)

def calculate_c(nx, ny, nz):
    cx = (math.sqrt(3)/2) * (d - (e * (1 - ((math.pow(nx,2) - (math.sqrt(3)*nx*ny))/(nz + 1)))))
    cy = -cx/math.sqrt(3)
    cz = h + ((e/2)*((math.sqrt(3)*nx) - ny))
    cm = math.sqrt(math.pow(cx, 2) + math.pow(cy, 2) + math.pow(cz, 2))
    c_theta = math.acos(((math.sqrt(3)*cx) - cy)/(2*cm)) + math.acos((math.pow(cm, 2) + math.pow(f, 2) - math.pow(g, 2))/(2*cm*f))
    return(c_theta)

def get_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]

    if hue >= 165:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

def Publisher():

    error_lists = [[], []]
    sum_error = [0, 0]   #x, y
    diff_error = [0, 0]
    kp = [0, 0]
    ki = [0, 0]
    kd = [0, 0]

    pub = rospy.Publisher('angles', String, queue_size=10)
    rospy.init_node('angle_pub_node', anonymous=False)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        global w, h
        ret, frame = source.read() 
        w = int(w) 
        h = int(h)
        frame = cv2.resize(frame, (w, h))

        frame = cv2.flip(frame, 1)
               
        #frame = calibrate(frame, camera_matrix, dist_coefs)
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lowerLimit, upperLimit = get_limits(color=yellow)

        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
        #y = cv2.bitwise_and(frame, frame, mask=mask)
        mask_ = Image.fromarray(mask)

        bbox = mask_.getbbox()

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            #frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

            x_centre_ball = (x1 + x2)//2
            y_centre_ball = (y1 + y2)//2

            x_centre_real = (x_origin - x_centre_ball) / 37.795
            y_centre_real = -(y_origin - y_centre_ball) / 37.795

            #frame = cv2.circle(frame, (x_origin,y_origin), radius=0, color=(0, 0, 255), thickness=5)
            #frame = cv2.circle(frame, (x_centre_ball,y_centre_ball), radius=0, color=(0, 0, 255), thickness=5)

            x_error = x_origin_real - x_centre_real
            y_error = y_origin_real - y_centre_real

            sum_error[0] += x_error
            sum_error[1] += y_error

            error_lists[0].append(x_error)
            error_lists[1].append(y_error)

            if(len(error_lists[0][0]) == 2):
                diff_error[0] = (error_lists[0][1] - error_lists[0][0])/0.02
                diff_error[1] = (error_lists[1][1] - error_lists[1][0])/0.02

            nx = -(kp[0]*x_error + ki[0]*sum_error[0] + kd[0]*diff_error[0])
            ny = -(kp[1]*y_error + ki[1]*sum_error[1] + kd[1]*diff_error[1])

            nmag = math.sqrt(math.pow(nx, 2) + math.pow(ny, 2) + 1)
            nx /= nmag
            ny /= nmag
            nz = 1/nmag

            theta_a = calculate_a(nx, ny, nz) * (180/math.pi) * (80/9)
            theta_b = calculate_b(nx, ny, nz) * (180/math.pi) * (80/9)
            theta_c = calculate_c(nx, ny, nz) * (180/math.pi) * (80/9)

            message = f'{theta_a} {theta_b} {theta_c}'

            #frame = cv2.putText(frame, message, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                
        else:
            message = 'None'

        pub.publish(message)
        rate.sleep()

    source.release() 
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try :
        Publisher()
    except rospy.ROSInterruptException:
        pass