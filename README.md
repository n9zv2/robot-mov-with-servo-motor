# robot-mov-with-servo-motor
![01](https://github.com/user-attachments/assets/22eb18af-90fc-462a-9653-030f99e6db1d)

![02](https://github.com/user-attachments/assets/8a1ec0c0-b7dd-4b44-bc9c-5589e852817c)


## Components
- **Arduino Uno**: A microcontroller board used to control the servo motors.
- **Breadboard**: Used for connecting multiple components without soldering.
- **Servo Motors**: A type of motor that allows for precise control of angular position.
- **Connecting Wires**: Used to make electrical connections between the Arduino, breadboard, and servo motors.

## Connections

### Power Connections
- The ground (GND) pin on the Arduino is connected to the negative rail on the breadboard.
- The 5V pin on the Arduino is connected to the positive rail on the breadboard.
- Each servo motor has three wires: ground (black/brown), power (red), and signal (various colors).
- The ground wires of all servo motors are connected to the negative rail on the breadboard.
- The power wires of all servo motors are connected to the positive rail on the breadboard.

### Signal Connections
- The signal wire of each servo motor is connected to a digital pin on the Arduino:
  - Servo 1 (leftmost) -> Digital Pin 9
  - Servo 2 -> Digital Pin 10
  - Servo 3 -> Digital Pin 11
  - Servo 4 -> Digital Pin 12
  - Servo 5 (rightmost) -> Digital Pin 13

### Detailed Wiring
- **Servo 1**:
  - Signal (Orange/Yellow) -> Digital Pin 9
  - Power (Red) -> Positive rail
  - Ground (Black/Brown) -> Negative rail
- **Servo 2**:
  - Signal (Orange/Yellow) -> Digital Pin 10
  - Power (Red) -> Positive rail
  - Ground (Black/Brown) -> Negative rail
- **Servo 3**:
  - Signal (Orange/Yellow) -> Digital Pin 11
  - Power (Red) -> Positive rail
  - Ground (Black/Brown) -> Negative rail
- **Servo 4**:
  - Signal (Orange/Yellow) -> Digital Pin 12
  - Power (Red) -> Positive rail
  - Ground (Black/Brown) -> Negative rail
- **Servo 5**:
  - Signal (Orange/Yellow) -> Digital Pin 13
  - Power (Red) -> Positive rail
  - Ground (Black/Brown) -> Negative rail


 ### The code
 ```cpp
#include <Servo.h>

// Define the servo motors for each joint
Servo base, shoulder, elbow, wrist1, wrist2, gripper;

// Define the angles for each joint
float theta1, theta2, theta3, theta4, theta5, theta6;

// Define the lengths of each link in the robot arm in cm
float L1 = 10;
float L2 = 10;
float L3 = 10;
float L4 = 10;
float L5 = 10;
float L6 = 10;

// Define the end effector position
float x = 0;
float y = 0;
float z = 0;
int pos = 0; 

void setup() {
  // Attach the servo motors to their respective pins
  base.attach(9);
  shoulder.attach(6);
  elbow.attach(5);
  wrist1.attach(3);
  wrist2.attach(10);
  gripper.attach(11);
  
  // Set the initial position of the servo motors
  base.write(90);
  shoulder.write(90);
  elbow.write(90);
  wrist1.write(90);
  wrist2.write(90);
  gripper.write(0);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Wait for input from the user
  if (Serial.available() > 0) {
    // Read the input from the user
    String input = Serial.readStringUntil('\n');

    // Parse the input into x, y, and z coordinates
    int comma1 = input.indexOf(",");
    int comma2 = input.indexOf(",", comma1+1);
    x = input.substring(0, comma1).toFloat(); // Reads the first value entered in the string
    y = input.substring(comma1+1, comma2).toFloat(); // Reads the second value entered after the first comma i.e. y coordinate
    z = input.substring(comma2+1).toFloat(); // Reads final value after the second comma i.e. z coordinate

    // Calculate the inverse kinematics
    // Equations are written on paper attached
    float r = sqrt(x*x + y*y);
    float d = sqrt(r*r + (z-L1)*(z-L1));
    theta1 = atan2(y, x);
    theta2 = atan2(z-L1, r) + acos((L3*L3 - L2*L2 - d*d)/(-2*L2*d));
    theta3 = atan2(d*sin(theta2), L2 + d*cos(theta2)) + atan2((z-L1), r);
    theta4 = atan2(-cos(theta2)*sin(theta3), -sin(theta2)) - theta2 - theta3;
    theta5 = atan2(cos(theta2)*cos(theta3)*cos(theta4) - sin(theta2)*sin(theta4), cos(theta4)*sin(theta3));
    theta6 = atan2(sin(theta5)*cos(theta2)*cos(theta3) + sin(theta2)*cos(theta5), cos(theta4));
    
    // Convert the angles to degrees and map them to the servo range
    int baseAngle = map(theta1*180/PI, 0, 180, 0, 90);
    int shoulderAngle = map(theta2*180/PI, 0, 180, 0, 90);
    int elbowAngle = map(theta3*180/PI, 0, 180, 0, 90);
    int wrist1Angle = map(theta4*180/PI, 0, 180, 0, 90);
    int wrist2Angle = map(theta5*180/PI, 0, 180, 0, 90);
    int gripperAngle = map(theta6*180/PI, 0, 180, 0, 0);
    
    // Write the angles to the servo motors
    for (pos = 0; pos <= baseAngle; pos += 1) { 
      base.write(pos);            
      delay(15);
    }
    shoulder.write(shoulderAngle);
    elbow.write(elbowAngle);
    wrist1.write(wrist1Angle);
    wrist2.write(wrist2Angle);
    gripper.write(gripperAngle);

    // Print the angles to the serial monitor
    Serial.print("Base Angle: ");
    Serial.print(baseAngle);
    Serial.print(" | Shoulder Angle: ");
    Serial.print(shoulderAngle);
    Serial.print(" | Elbow Angle: ");
    Serial.print(elbowAngle);
    Serial.print(" | Wrist 1 Angle: ");
    Serial.print(wrist1Angle);
    Serial.print(" | Wrist 2 Angle: ");
    Serial.print(wrist2Angle);
    Serial.print(" | Gripper Angle: ");
    Serial.println(gripperAngle);
  }
}

