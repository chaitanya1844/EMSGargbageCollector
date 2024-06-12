#include <Arduino.h>
#include <SPI.h>
int getDistance();
void dropItem();
void pickUpItem();
void setServoPosition(uint8_t servoNum, uint16_t angle);

#define rightPWM 9 //ENA
#define rightFWD 4 //IN1
#define rightBCK 5 //IN2
#define leftPWM 3  //ENA
#define leftFWD 7  //IN3
#define leftBCK 6  //IN4
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
 
#define SERVOMIN_BASE   150     // Minimum pulse width for base servo
#define SERVOMAX_BASE   600     // Maximum pulse width for base servo
#define SERVOMIN_SHOULDER  150  // Minimum pulse width for shoulder servo
#define SERVOMAX_SHOULDER  600  // Maximum pulse width for shoulder servo
#define SERVOMIN_ELBOW  150     // Minimum pulse width for elbow servo
#define SERVOMAX_ELBOW  600     // Maximum pulse width for elbow servo
#define SERVOMIN_YAW  150       // Minimum pulse width for yaw servo
#define SERVOMAX_YAW  600       // Maximum pulse width for yaw servo
#define SERVOMIN_PITCH  150     // Minimum pulse width for pitch servo
#define SERVOMAX_PITCH  600     // Maximum pulse width for pitch servo
#define SERVOMIN_GRIPPER  150   // Minimum pulse width for gripper servo
#define SERVOMAX_GRIPPER  600   // Maximum pulse width for gripper servo
#define SERVO_FREQ 50           // PWM frequency for the servo module

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 
char command;

void setup() {
  Serial.begin(9600);
  pinMode(rightFWD, OUTPUT);
  pinMode(rightBCK, OUTPUT);
  pinMode(leftFWD, OUTPUT);
  pinMode(leftBCK, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency for the servo module

  // Configure initial positions of servo motors
  setServoPosition(4, 55); // Base servo (rotation)
  setServoPosition(5, 165); // Shoulder servo
  setServoPosition(6, 90); // Elbow servo
  setServoPosition(7, 90); // Yaw servo
  setServoPosition(8, 180); // Pitch servo
  setServoPosition(9, 100); // Gripper servo

}

void loop() {
  if (Serial.available()) {
    command = Serial.read();
    Serial.print(command);
  
    if (command == 'F') {
      digitalWrite(rightFWD, HIGH);
      digitalWrite(rightBCK, LOW);
      digitalWrite(leftFWD, HIGH);
      digitalWrite(leftBCK, LOW);
      analogWrite(rightPWM, 255);
      analogWrite(leftPWM, 255);
    }
    else if (command == 'O') {
      digitalWrite(rightFWD, LOW);
      digitalWrite(rightBCK, LOW);
      digitalWrite(leftFWD, LOW);
      digitalWrite(leftBCK, LOW);
      analogWrite(rightPWM, 0);
      analogWrite(leftPWM, 0);
    }
    else if (command == 'B') {
      digitalWrite(rightFWD, LOW);
      digitalWrite(rightBCK, HIGH);
      digitalWrite(leftFWD, LOW);
      digitalWrite(leftBCK, HIGH);
      analogWrite(rightPWM, 255);
      analogWrite(leftPWM, 255);
    }
    else if (command == 'S') {
      digitalWrite(rightFWD, LOW);
      digitalWrite(rightBCK, HIGH);
      digitalWrite(leftFWD, HIGH);
      digitalWrite(leftBCK, LOW);
      analogWrite(rightPWM, 0);
      analogWrite(leftPWM, 255);
    }
    else if (command == 'C') {
      digitalWrite(rightFWD, HIGH);
      digitalWrite(rightBCK, LOW);
      digitalWrite(leftFWD, LOW);
      digitalWrite(leftBCK, HIGH);
      analogWrite(rightPWM, 255);
      analogWrite(leftPWM, 255);
    }
    else if (command == 'A') {
      digitalWrite(rightFWD, LOW);
      digitalWrite(rightBCK, LOW);
      digitalWrite(leftFWD, LOW);
      digitalWrite(leftBCK, LOW);
      analogWrite(rightPWM, 0);
      analogWrite(leftPWM, 0);
    }
    else if (command=='L'){
      pickUpItem();
    }
  }
}

void setServoPosition(uint8_t servoNum, uint16_t angle) {
  // Convert angle to pulse width
  uint16_t pulseWidth = map(angle, 0, 180, SERVOMIN_BASE, SERVOMAX_BASE);
  // Set pulse width for the specified servo
  pwm.setPWM(servoNum, 0, pulseWidth);
}

void pickUpItem() {
  
  // Move the arm to pick up the item
  //setServoPosition(4, 45); // Base rotates (adjust as needed)
  setServoPosition(9, 10);
  setServoPosition(5, 120); // Shoulder moves down
  setServoPosition(6, 105); // Elbow moves down
  setServoPosition(7, 60); // Yaw servo (adjust as needed)
  setServoPosition(8, 140); // Pitch servo (adjust as needed)
  delay(1000); // Delay for arm to move
  
  // Close the gripper to grab the item
  setServoPosition(9, 100); // Gripper closes
  delay(1000); // Delay for gripper to close
  
  // Lift the arm up with the item
  setServoPosition(5, 165); // Shoulder moves up
  setServoPosition(6, 90); // Elbow moves up
  setServoPosition(8,40);
  delay(1000); // Delay for arm to move
  
  // Return the arm to the initial position
  setServoPosition(4, 55); // Base returns to initial position
  //setServoPosition(9, 20); // Gripper opens
}


