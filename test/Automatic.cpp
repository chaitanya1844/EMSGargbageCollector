#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
int getDistance();
void dropItem();
void pickUpItem();
void setServoPosition(uint8_t servoNum, uint16_t angle);
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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Create an object for the PWM servo driver

#define TRIGGER_PIN 13//ultrasonic pin
#define ECHO_PIN 12
//#define MAX_DISTANCE 200

#define rightPWM 9 //ENA
#define rightFWD 4 //IN!
#define rightBCK 5 //IN2
#define leftPWM 3  //ENA
#define leftFWD 7  //IN3
#define leftBCK 6  //IN4

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency for the servo module

  // Configure initial positions of servo motors
  setServoPosition(4, 55); // Base servo (rotation)
  setServoPosition(5, 165); // Shoulder servo
  setServoPosition(6, 90); // Elbow servo
  setServoPosition(7, 90); // Yaw servo
  setServoPosition(8, 180); // Pitch servo
  setServoPosition(9, 100); // Gripper servo

  pinMode(rightFWD, OUTPUT);
  pinMode(rightBCK, OUTPUT);
  pinMode(leftFWD, OUTPUT);
  pinMode(leftBCK, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  delay(3000);
}



void loop() {
  
  int loopcon=0;
  digitalWrite(rightFWD, HIGH);
  digitalWrite(rightBCK, LOW);
  digitalWrite(leftFWD, HIGH);
  digitalWrite(leftBCK, LOW);
  analogWrite(rightPWM, 200);
  analogWrite(leftPWM, 200); 
  int distance =getDistance();
  Serial.print("\n");
  Serial.print(distance);
  if (distance <= 15 && distance>=7) {  // Adjust this value as per your requirement
  digitalWrite(rightFWD, LOW);
  digitalWrite(rightBCK, HIGH);
  digitalWrite(leftFWD, LOW);
  digitalWrite(leftBCK, HIGH);
  analogWrite(rightPWM,255);
  analogWrite(leftPWM,255);
  digitalWrite(rightFWD, LOW);
  digitalWrite(rightBCK, LOW);
  digitalWrite(leftFWD, LOW);
  digitalWrite(leftBCK, LOW);

  while(loopcon==0){
    delay(500);
    pickUpItem();
    delay(1000);
    for(int j=0;j<100;j++){
    distance=getDistance();
    Serial.print("\n");
    Serial.print(distance);
    if(distance>20){
          loopcon++;}}}
  delay(2000);
  for(int i=0;i<100;i++){
  digitalWrite(rightFWD, LOW);
  digitalWrite(rightBCK, HIGH);
  digitalWrite(leftFWD, LOW);
  digitalWrite(leftBCK, HIGH);
  analogWrite(rightPWM,200);
  analogWrite(leftPWM,200);
  delay(20);
  }

    // Stop motors if the backward motion duration has passed
    while(1){
    digitalWrite(rightFWD, LOW);
    digitalWrite(rightBCK, LOW);
    digitalWrite(leftFWD, LOW);
    digitalWrite(leftBCK, LOW);
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(1000);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(1000); }
  }

}

  

//||||||||||||||||||||||||||||||||||||| CUSTOM FUNCTIONS |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

//|||||||||||||||||| ROBOTIC ARM ||||||||||||||||||||
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
  
  
  setServoPosition(9, 100); // Gripper closes
  delay(1000); // Delay for gripper to close
  
  
  setServoPosition(5, 165); // Shoulder moves up
  setServoPosition(6, 90); // Elbow moves up
  setServoPosition(8,40);
  delay(1000); // Delay for arm to move
  
  // Return the arm to the initial position
  setServoPosition(4, 55); // Base returns to initial position
  //setServoPosition(9, 20); // Gripper opens
}

void dropItem(){
  setServoPosition(9, 10);
  setServoPosition(5, 120); // Shoulder moves down
  setServoPosition(6, 105); // Elbow moves down
  setServoPosition(7, 60); // Yaw servo (adjust as needed)
  setServoPosition(8, 140); // Pitch servo (adjust as needed)
  delay(1000); // Delay for arm to move
  
  
  setServoPosition(9, 100); // Gripper closes
  delay(1000); // Delay for gripper to close
  
  
  setServoPosition(5, 165); // Shoulder moves up
  setServoPosition(6, 90); // Elbow moves up
  setServoPosition(8,40);
  delay(1000); // Delay for arm to move
  
  // Return the arm to the initial position
  setServoPosition(4, 55);
}



//|||||||||||||||||||||||||||||||||||||||| CAR MOTORS ||||||||||||||||||||||||||||||||||||||||||||||||||||

int getDistance() {
  // Send a short ultrasonic pulse to trigger the sensor
  int dis=200;
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance based on the speed of sound (340 m/s) and the duration of the echo pulse
  dis = duration * 0.034 / 2; // Divide by 2 because the sound travels back and forth

  return dis;
}

