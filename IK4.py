#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from std_msgs.msg import String
import math


vcamno = 1

# Constants
Pz = 1
L = [5, 4.845, 8.72, 7.94]

Pmx = None
Pmz = None
sqrt3 = math.sqrt(3)  # Precompute sqrt(3)

# Function to calculate theta_a
def calculate_a(nx, ny, nz):
    n = [nx, ny, nz]
    global Pmx, Pmz
    A = (L[0] + L[1]) / Pz
    B = (Pz ** 2 + L[2] ** 2 - (L[0] + L[1]) ** 2 - L[3] ** 2) / (2 * Pz)
    C = A ** 2 + 1
    D = 2 * (A * B - (L[0] + L[1]))
    E = B ** 2 + (L[0] + L[1]) ** 2 - L[2] ** 2

    # Avoid negative values inside the square root
    discriminant = D ** 2 - 4 * C * E
    if discriminant < 0:
        raise ValueError("Negative discriminant in theta_a calculation")

    Pmx = (-D + math.sqrt(discriminant)) / (2 * C)
    Pmz = math.sqrt(abs(L[2] ** 2 - Pmx ** 2 + 2 * (L[0] + L[1]) * Pmx - (L[0] + L[1]) ** 2))

    # Calculate servo angle a
    a_m_x = (L[3] / math.sqrt(n[0] ** 2 + n[2] ** 2)) * n[2]
    a_m_z = Pz + (L[3] / math.sqrt(n[0] ** 2 + n[2] ** 2)) * -n[0]

    A_m = [a_m_x, 0, a_m_z]
    A = (L[0] - A_m[0]) / A_m[2]
    B = (A_m[0] ** 2 + A_m[1] ** 2 + A_m[2] ** 2 - L[2] ** 2 - L[0] ** 2 + L[1] ** 2) / (2 * A_m[2])
    C = A ** 2 + 1
    D = 2 * (A * B - L[0])
    E = B ** 2 + L[0] ** 2 - L[1] ** 2

    # Avoid negative values inside the square root
    discriminant = D ** 2 - 4 * C * E
    if discriminant < 0:
        raise ValueError("Negative discriminant in theta_a servo calculation")

    ax = (-D + math.sqrt(discriminant)) / (2 * C)
    az = math.sqrt(abs(L[1] ** 2 - ax ** 2 + 2 * L[0] * ax - L[0] ** 2))
    if a_m_z < Pmz:
        az = -az

    A_2 = [ax, 0, az]
    theta_a = 90 - math.degrees(math.atan2(A_2[0] - L[0], A_2[2]))
    return theta_a

# Function to calculate theta_b
def calculate_b(nx, ny, nz):
    n = [nx, ny, nz]
    global Pmx, Pmz
    sqrt_part = math.sqrt(n[0] ** 2 + 3 * n[1] ** 2 + 4 * n[2] ** 2 + 2 * sqrt3 * n[0] * n[1])

    b_m_x = (L[3] / sqrt_part) * (-n[2])
    b_m_y = (L[3] / sqrt_part) * (-sqrt3 * n[2])
    b_m_z = Pz + (L[3] / sqrt_part) * (sqrt3 * n[1] + n[0])

    B_m = [b_m_x, b_m_y, b_m_z]

    A = -(B_m[0] + sqrt3 * B_m[1] + 2 * L[0]) / B_m[2]
    B = (B_m[0] ** 2 + B_m[1] ** 2 + B_m[2] ** 2 + L[1] ** 2 - L[2] ** 2 - L[0] ** 2) / (2 * B_m[2])
    C = A ** 2 + 4
    D = 2 * A * B + 4 * L[0]
    E = B ** 2 + L[0] ** 2 - L[1] ** 2

    discriminant = D ** 2 - 4 * C * E
    if discriminant < 0:
        raise ValueError("Negative discriminant in theta_b calculation")

    x = (-D - math.sqrt(discriminant)) / (2 * C)
    y = sqrt3 * x
    z = math.sqrt(abs(L[1] ** 2 - 4 * x ** 2 - 4 * L[0] * x - L[0] ** 2))
    if b_m_z < Pmz:
        z = -z

    B_2 = [x, y, z]
    theta_b = 90 - math.degrees(math.atan2(math.sqrt(B_2[0] ** 2 + B_2[1] ** 2) - L[0], B_2[2]))
    return theta_b

# Function to calculate theta_c
def calculate_c(nx, ny, nz):
    n = [nx, ny, nz]
    global Pmx, Pmz
    sqrt_part = math.sqrt(n[0] ** 2 + 3 * n[1] ** 2 + 4 * n[2] ** 2 - 2 * sqrt3 * n[0] * n[1])

    c_m_x = (L[3] / sqrt_part) * (-n[2])
    c_m_y = (L[3] / sqrt_part) * (sqrt3 * n[2])
    c_m_z = Pz + (L[3] / sqrt_part) * (-sqrt3 * n[1] + n[0])

    C_m = [c_m_x, c_m_y, c_m_z]

    A = -(C_m[0] - sqrt3 * C_m[1] + 2 * L[0]) / C_m[2]
    B = (C_m[0] ** 2 + C_m[1] ** 2 + C_m[2] ** 2 + L[1] ** 2 - L[2] ** 2 - L[0] ** 2) / (2 * C_m[2])
    C = A ** 2 + 4
    D = 2 * A * B + 4 * L[0]
    E = B ** 2 + L[0] ** 2 - L[1] ** 2

    discriminant = D ** 2 - 4 * C * E
    if discriminant < 0:
        raise ValueError("Negative discriminant in theta_c calculation")

    x = (-D - math.sqrt(discriminant)) / (2 * C)
    y = -sqrt3 * x
    z = math.sqrt(abs(L[1] ** 2 - 4 * x ** 2 - 4 * L[0] * x - L[0] ** 2))
    if c_m_z < Pmz:
        z = -z

    C_2 = [x, y, z]
    theta_c = 90 - math.degrees(math.atan2(math.sqrt(C_2[0] ** 2 + C_2[1] ** 2) - L[0], C_2[2]))
    return theta_c
