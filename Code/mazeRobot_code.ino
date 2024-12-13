#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float gX, gY, gZ, mDirection, mX, mY, mZ;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *myMotorLeft = AFMS.getMotor(2);
Adafruit_DCMotor *myMotorRight = AFMS.getMotor(1);

const int TURN_DELAY = 900;
const int TURN_AROUND_DELAY = 1530;
const int ADJUST_DELAY = 150;

const int TURN_OUTER_SPEED = 70;
const int TURN_INNER_SPEED = 0;
const int FORWARD_SPEED = 70;
const int TURN_AROUND_SPEED = 50;
const int ADJUST_INNER_SPEED = 55;

const int LEFT_ECHO = 6;
const int LEFT_TRIG = 5;
const int FRONT_ECHO = 8;
const int FRONT_TRIG = 9;
const int RIGHT_ECHO = 12;
const int RIGHT_TRIG = 13;

const int DISTANCE_TO_WALL = 35;
const int DISTANCE_TO_FRONT_WALL = 20;
const int IDEAL_DISTANCE_TO_WALL = 10;

const float Kp = 1.0;  // Proportional gain
const float Ki = 0.02;  // Integral gain
const float Kd = 0.3;  // Derivative gain

float previousError = 0;
float integral = 0;
float dt = 0.1; // Time difference for each loop iteration
float setpoint = 10;

void setup() {

  Serial.begin(115200);
  while (!Serial);
  
  bme.begin();

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // put your setup code here, to run once:
  AFMS.begin();
  myMotorLeft->setSpeed(FORWARD_SPEED);
  myMotorRight->setSpeed(FORWARD_SPEED);
  myMotorLeft->run(FORWARD);
  myMotorRight->run(FORWARD);

  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (wallOnLeft()) {
    if (wallInFront()) {
      if (wallOnRight()) {
        turnAround();
        previousError = 0;
        integral = 0;
      }
      else {
        turnRight();
        previousError = 0;
        integral = 0;
      }
    }
    else forward();
    
    float distance = distanceFunktio(LEFT_TRIG, LEFT_ECHO);
    float error = setpoint - distance;
    
    // Proportional term
    float P = Kp * error;
    
    // Integral term
    integral += error * dt;
    float I = Ki * integral;
    
    // Derivative term
    float derivative = (error - previousError) / dt;
    float D = Kd * derivative;
    
    // PID output
    float output = P + I + D;
    
    // Use the output to control the motors (speed adjustment)
    adjustMotorSpeed(output);

    Serial.println("Adjusting... ");

    previousError = error;


  }
  else {
    turnLeft();
    previousError = 0;
    integral = 0;
  }
  
  delay(dt*1000);
}


void adjustMotorSpeed(float pidOutput) {
    // Assume two motors: Left and Right
    // Adjust the motor speeds based on the PID output
   
    int baseSpeed = FORWARD_SPEED;  // Base motor speed
    
    // If the PID output is positive, the robot is too far from the wall, so slow down the right motor
    if (pidOutput > 0) {
        myMotorLeft->setSpeed(baseSpeed);
        myMotorRight->setSpeed(baseSpeed - pidOutput); 
    } 
    // If the PID output is negative, the robot is too close, so slow down the left motor
    else {
        myMotorLeft->setSpeed(baseSpeed + pidOutput);
        myMotorRight->setSpeed(baseSpeed);
    }
}


void turnLeft() {
  Serial.println("Turning left.");
  delay(400);
  myMotorLeft->setSpeed(TURN_INNER_SPEED);
  delay(TURN_DELAY);
  myMotorLeft->setSpeed(FORWARD_SPEED);
  delay(700);
}

void turnRight() {
  Serial.println("Turning right.");
  myMotorRight->setSpeed(TURN_INNER_SPEED);
  delay(TURN_DELAY);
  myMotorRight->setSpeed(FORWARD_SPEED);
  delay(500);
}

void turnAround() {
  Serial.println("Turning around.");
  myMotorLeft->setSpeed(TURN_AROUND_SPEED);
  myMotorRight->setSpeed(TURN_AROUND_SPEED);
  myMotorRight->run(BACKWARD);
  delay(TURN_AROUND_DELAY);

  myMotorLeft->setSpeed(FORWARD_SPEED);
  myMotorRight->setSpeed(FORWARD_SPEED);
  myMotorRight->run(FORWARD);
  delay(500);
}

void forward() {
  Serial.println("Continuing forward. ");
  myMotorLeft->setSpeed(FORWARD_SPEED);
  myMotorRight->setSpeed(FORWARD_SPEED);
}
/**
void adjust() {
  long distance = distanceFunktio(LEFT_TRIG, LEFT_ECHO);
  if (distance < IDEAL_DISTANCE_TO_WALL) {
    myMotorRight->setSpeed(ADJUST_INNER_SPEED);
    delay(ADJUST_DELAY);
    myMotorRight->setSpeed(FORWARD_SPEED);
  }
  if (distance > IDEAL_DISTANCE_TO_WALL) {
    myMotorLeft->setSpeed(ADJUST_INNER_SPEED);
    delay(ADJUST_DELAY);
    myMotorLeft->setSpeed(FORWARD_SPEED);
  }
}*/


bool wallOnLeft() {
  long distance = distanceFunktio(LEFT_TRIG, LEFT_ECHO);
  Serial.print("left ");
  Serial.println(distance);
  if (distance < DISTANCE_TO_WALL)
    return true;
  else return false;
}

bool wallInFront() {
  long distance = distanceFunktio(FRONT_TRIG, FRONT_ECHO);
  Serial.print("front ");
  Serial.println(distance);
  if (distance < DISTANCE_TO_FRONT_WALL)
    return true;
  else return false;
}

bool wallOnRight() {
  long distance = distanceFunktio(RIGHT_TRIG, RIGHT_ECHO);
  Serial.print("right ");
  Serial.println(distance);
  if (distance < DISTANCE_TO_WALL)
    return true;
  else return false;
}

int distanceFunktio(int trig, int echo) {
  long duration, distance;
  
  // ultraäänipulssi – ultrasonic pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  duration = pulseIn(echo, HIGH);
  
  // etäisyyden laskeminen – calculating the distance
  distance = duration * 0.034 / 2;

  return distance;
}