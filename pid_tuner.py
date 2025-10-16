#!/usr/bin/python3
import time

# File path where PID values will be stored
pid_file = "pid_values.txt"

def update_pid(kp, ki, kd):
    with open(pid_file, 'w') as file:
        file.write(f"{kp},{ki},{kd}")

if __name__ == "__main__":
    print("PID Tuning Script Running.")
    while True:
        try:
            # Get new PID values from user
            kp = float(input("Enter kp value: "))
            ki = float(input("Enter ki value: "))
            # ki = 0
            kd = float(input("Enter kd value: "))
            # kd = 0

            # Update the PID values by writing to the file
            update_pid(kp, ki, kd)
            print(f"Updated PID values: kp={kp}, ki={ki}, kd={kd}")

        except ValueError:
            print("Invalid input. Please enter numeric values.")

        time.sleep(1)
