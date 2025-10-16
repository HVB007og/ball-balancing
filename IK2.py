#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math
vcamno = 1


# New constants:
# l = 7.94
# L1 = 8.72
# L2 = 4.845
# L3  = 5
# h = 14
# # h = -17
Pz = 1
L = [5,4.845,8.72,7.94]
Pmx = None
Pmz = None

# Function to calculate theta_a
def calculate_a(nx, ny, nz):
    n = [nx,ny,nz]
    global Pmx, Pmz
    A = (L[0]+L[1])/Pz
    B = (Pz**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*Pz)
    C = A**2+1
    D = 2*(A*B-(L[0]+L[1]))
    E = B**2+(L[0]+L[1])**2-L[2]**2
    Pmx = (-D+math.sqrt(abs(D**2-4*C*E)))/(2*C)
    Pmz = math.sqrt(abs(L[2]**2-Pmx**2+2*(L[0]+L[1])*Pmx-(L[0]+L[1])**2))
    #サーボaの角度導出
    a_m_x = (L[3]/(math.sqrt(abs(n[0]**2 + n[2]**2))))*(n[2])
    a_m_y = 0
    a_m_z = Pz + (L[3]/(math.sqrt(abs(n[0]**2 + n[2]**2))))*(-n[0])
    A_m = [a_m_x, a_m_y, a_m_z]
    A = (L[0]-A_m[0])/A_m[2]
    B = (A_m[0]**2+A_m[1]**2+A_m[2]**2-L[2]**2-L[0]**2+L[1]**2)/(2*A_m[2])
    C = A**2+1
    D = 2*(A*B-L[0])
    E = B**2+L[0]**2-L[1]**2
    ax = (-D+math.sqrt(abs(D**2-4*C*E)))/(2*C)
    ay = 0
    az = math.sqrt(abs(L[1]**2-ax**2+2*L[0]*ax-L[0]**2))
    if (a_m_z < Pmz):
        az = -az
    A_2 = [ax, ay, az]
    theta_a = 90 - math.degrees(math.atan2(A_2[0]-L[0], A_2[2]))
    return theta_a




# Function to calculate theta_b
def calculate_b(nx, ny, nz):
    n = [nx,ny,nz]
    global Pmx, Pmz
    b_m_x = (L[3]/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(abs(3))*n[0]*n[1]))))*(-n[2])
    b_m_y = (L[3]/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(abs(3))*n[0]*n[1]))))*(-math.sqrt(abs(3))*n[2])
    b_m_z = Pz + (L[3]/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1]))))*(math.sqrt(3)*n[1]+n[0])
    B_m = [b_m_x, b_m_y, b_m_z]

    A = -(B_m[0]+math.sqrt(abs(3))*B_m[1]+2*L[0])/B_m[2]
    B = (B_m[0]**2+B_m[1]**2+B_m[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*B_m[2])
    C = A**2+4
    D = 2*A*B+4*L[0]
    E = B**2+L[0]**2-L[1]**2
    x = (-D-math.sqrt(abs(D**2-4*C*E)))/(2*C)
    y = math.sqrt(abs(3))*x
    z = math.sqrt(abs(L[1]**2-4*x**2-4*L[0]*x-L[0]**2))
    if (b_m_z < Pmz):
        z = -z
    B_2 = [x, y, z]
    theta_b = 90 - math.degrees(math.atan2(math.sqrt(abs(B_2[0]**2+B_2[1]**2))-L[0], B_2[2]))
    return theta_b


# Function to calculate theta_c
def calculate_c(nx, ny, nz):
    n = [nx,ny,nz]
    global Pmx, Pmz
    c_m_x = (L[3]/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(abs(3))*n[0]*n[1]))))*(-n[2])
    c_m_y = (L[3]/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(abs(3))*n[0]*n[1]))))*(math.sqrt(abs(3))*n[2])
    c_m_z = Pz + (L[3]/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(abs(3))*n[0]*n[1]))))*(-math.sqrt(abs(3))*n[1]+n[0])
    C_m = [c_m_x, c_m_y, c_m_z]

    A = -(C_m[0]-math.sqrt(abs(3))*C_m[1]+2*L[0])/C_m[2]
    B = (C_m[0]**2+C_m[1]**2+C_m[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*C_m[2])
    C = A**2+4
    D = 2*A*B+4*L[0]
    E = B**2+L[0]**2-L[1]**2
    x = (-D-math.sqrt(abs(D**2-4*C*E)))/(2*C)
    y = -math.sqrt(abs(3))*x
    z = math.sqrt(abs(L[1]**2-4*x**2-4*L[0]*x-L[0]**2))
    if (c_m_z < Pmz):
        z = -z
    C_2 = [x, y, z]
    theta_c = 90 - math.degrees(math.atan2(math.sqrt(abs(C_2[0]**2+C_2[1]**2))-L[0], C_2[2]))
    return theta_c
