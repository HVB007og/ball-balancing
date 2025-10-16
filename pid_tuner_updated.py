#!/usr/bin/python3
import time
import os

# File path where PID values will be stored
pid_file = "pid_values.txt"

def read_pid():
    # Check if the PID file exists and read its content
    if os.path.exists(pid_file):
        with open(pid_file, 'r') as file:
            content = file.read().strip()
            if content:
                kp, ki, kd = map(float, content.split(','))
                return kp, ki, kd
    # Default values if file is empty or doesn't exist
    return 0.0, 0.0, 0.0

def update_pid(kp, ki, kd):
    with open(pid_file, 'w') as file:
        file.write(f"{kp},{ki},{kd}")

if __name__ == "__main__":
    print("PID Tuning Script Running.")
    while True:
        try:
            # Read and display old PID values
            old_kp, old_ki, old_kd = read_pid()
            print(f"Current PID values: kp={old_kp}, ki={old_ki}, kd={old_kd}")

            # Get new PID values from user in one line
            pid_input = input("Enter new PID values (kp,ki,kd) separated by commas: ")
            kp, ki, kd = map(float, pid_input.split(','))

            # Update the PID values by writing to the file
            update_pid(kp, ki, kd)
            # print(f"Updated PID values: kp={kp}, ki={ki}, kd={kd}")

        except ValueError:
            print("Invalid input. Please enter numeric values separated by commas (e.g., 1.0,0.5,0.1).")

        time.sleep(1)
