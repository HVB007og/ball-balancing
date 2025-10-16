place files in a project in a ros workspace, place the contents of "Place_In_Workspace_Folder" directory in the workspace.
and run following commands.

Commands:
-> roscore in seperate terminal

-> rosrun proj vision13.py 


After connecting arduino, in new terminal:

-> rosrun rosserial_python serial_node.py /dev/ttyUSB0


Troubleshooting:

if camera not detecting, change camera number iin vision13.py file (line 13).
find cameras by: -> ls /dev/video*
put the last number in vcamno.



Arduino code:
****************************************************************************
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

int speed = 200 * 20;  // Stepper speed (steps/sec)
int acc = 4000;
AccelStepper stepperA(AccelStepper::DRIVER, 2, 5);  // (driver type, STEP, DIR) Driver A
AccelStepper stepperB(AccelStepper::DRIVER, 4, 3);  // (driver type, STEP, DIR) Driver B
AccelStepper stepperC(AccelStepper::DRIVER, 7, 6);  // (driver type, STEP, DIR) Driver C

long target_positions[3] = {0, 0, 0};  // Target positions for stepper motors

bool checkvalid(long positions[3]) {
  for (int i = 0; i < 3; i++) {
    if (positions[i] < -960 || positions[i] > 0) {
      return false;
    }
  }
  return true;
}

void move_steppers() {
  // Move each stepper to its respective position
  stepperA.moveTo(target_positions[0]);
  stepperB.moveTo(target_positions[1]);
  stepperC.moveTo(target_positions[2]);
}

// ROS callback for incoming position data
void positionCallback(const std_msgs::String& msg) {
  String input = msg.data;
  nh.loginfo(input.c_str());

  if (input != "None") {
    int index = 0;
    int lastIndex = 0;
    
    for (int i = 0; i < 3; i++) {
      index = input.indexOf(',', lastIndex);
      if (index == -1 && i == 2) {
        index = input.length();  // Last value (no comma at the end)
      }
      target_positions[i] = input.substring(lastIndex, index).toInt();
      lastIndex = index + 1;
    }

    if (checkvalid(target_positions)) {
      move_steppers();
      nh.loginfo("Moving to target positions");
    } else {
      nh.loginfo("Invalid positions received");
    }
  }
}

// ROS subscriber
ros::Subscriber<std_msgs::String> sub("angles", positionCallback);

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);

  // Set maximum speed and acceleration for the steppers
  stepperA.setMaxSpeed(speed);
  stepperB.setMaxSpeed(speed);
  stepperC.setMaxSpeed(speed);
  stepperA.setAcceleration(acc);
  stepperB.setAcceleration(acc);
  stepperC.setAcceleration(acc);

  nh.loginfo("Setup complete, waiting for commands...");
}

void loop() {
  nh.spinOnce();  // Handle ROS communication

  // Update the stepper positions
  stepperA.run();
  stepperB.run();
  stepperC.run();
}
************************************************************************
