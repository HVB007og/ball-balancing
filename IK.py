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
    # point on table
    n = [nx,ny,nz]
    a = (L/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1]))))*(-n[2])
    b = (L/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1]))))*(-math.sqrt(3)*n[2])
    c = h + (L3/(math.sqrt(abs(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1]))))*(math.sqrt(3)*n[1]+n[0])
        
    # Intermediate Simplifications

    A = -(a+math.sqrt(3)*b+2*L3)/c
    B = (a**2+b**2+c**2+L2**2-L1**2-L3**2)/(2*c)
    C = A**2+4
    D = 2*A*B+4*L3
    E = B**2+L3**2-L2**2

    # pin joint position
    d = (-D-math.sqrt(abs(D**2-4*C*E)))/(2*C)
    e = math.sqrt(3)*d
    f = math.sqrt(abs(L2**2-4*d**2-4*L3*d-L3**2))
    b_theta = 90 - math.degrees(math.atan2(math.sqrt(abs(d**2+e**2))-L3, f))
    return b_theta

# Function to calculate theta_c
def calculate_c(nx, ny, nz):
    
    c_m_x = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(-n[2])
    c_m_y = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(math.sqrt(3)*n[2])
    c_m_z = Pz + (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(-math.sqrt(3)*n[1]+n[0])
    C_m = [c_m_x, c_m_y, c_m_z]

    A = -(C_m[0]-math.sqrt(3)*C_m[1]+2*L[0])/C_m[2]
    B = (C_m[0]**2+C_m[1]**2+C_m[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*C_m[2])
    C = A**2+4
    D = 2*A*B+4*L[0]
    E = B**2+L[0]**2-L[1]**2
    x = (-D-math.sqrt(D**2-4*C*E))/(2*C)
    y = -math.sqrt(3)*x
    z = math.sqrt(L[1]**2-4*x**2-4*L[0]*x-L[0]**2)
    if (c_m_z < Pmz):
        z = -z
    C_2 = [x, y, z]
    theta_c = 90 - math.degrees(math.atan2(math.sqrt(C_2[0]**2+C_2[1]**2)-L[0], C_2[2]))
    c_theta  = 0
    return c_theta
