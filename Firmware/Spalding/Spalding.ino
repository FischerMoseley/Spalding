/*
Spaulding is a ballbot-inspired skateboard, named after the basketball in his drivetrain.
She uses angled three stepper motors on top of a basketball to roll around, allowing her
to rotate in the Z-axis while simultaneously translating in the X and Y axes. Currently,
she's just a small proof of concept, but if the controls theory works out she'll be 
turned into a final project for 6.1311, a power electronics lab class at MIT!

Firmware by Fischer Moseley. Feel free to use it as you wish!

Here theta is defined as a parameter that represents the declination angle of the
motor from the z-axis. You choose this angle, but the microcontroller doesn't calculate
 its sine or cosine, because that would take too long, use too much memory and precision
 would be lost. Edit these #defines with the sin/cos/tan/sec/csc/cot of the angle you wish
*/

#include <AccelStepper.h>
#include <AFMotor.h>
#include <MultiStepper.h>
#include <PID_v1.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

//using theta = pi/4
#define sec_theta 1.41421356237
#define csc_theta 1.41421356237
#define wheel_radius 5 //wheel radius in cm
#define ball_radius 12.13 //ball radius in cm

#define sqrt3 1.73205080757 

/*--------------- State Variables -----------------*/

float gyroPosition[3]; //literally have no idea how to get an euler output, just quaternions at the moment
float gyroRate[3]; //in the form x-axis rad/sec, y-axis, rad/sec z-axis rad/sec
uint16_t stateVector[3]; //in the form x-translation, y-translation, z-rotation
uint16_t stepperSpeeds[3]; //in the form motor1-rpm, motor2-rpm, motor3-rpm

/*------------- Hardware Configuration ----------- */
//Stepper motor configuration
#define StepsPerRotation 200
#define StepperMaxSpeed 100
AccelStepper stepper1(1, 3, 2);
AccelStepper stepper2(1, 5, 4);
AccelStepper stepper3(1, 6, 7);

//IMU configuration
BNO080 IMU;

//PID configuration
double xInput, xOutput, xSetPoint;
double yInput, yOutput, ySetPoint;

//define tuning parameters
double kP = 100;
double kI = 0;
double kD = 0;

//Specify links and initial tuning parameters
PID xAxisPID(&xInput, &xOutput, &xSetpoint, kP, kI, kD);
PID yAxisPID(&yInput, &yOutput, &ySetpoint, kp, kI, kD);

void updateGyroStateVariables(float* gyroRate){
  if(IMU.dataAvailable()){
    gyroRate[0] = IMU.getGyroX();
    gyroRate[1] = IMU.getGyroY();
    gyroRate[2] = IMU.getGyroZ();
  }
}

void updateStepperSpeeds(float* stateVector, float* stepperSpeeds){
  stepperSpeeds[0] = (-2*x*csc_theta - w_z*ball_radius*sec_theta)/(3*wheel_radius);
  stepperSpeeds[1] = ((x + sqrt3*y)*csc_theta - w_z*ball_radius*sec_theta)/(3*wheel_radius);
  stepperSpeeds[2] = ((x - sqrt3*y)*csc_theta - w_z*ball_radius*sec_theta)/(3*wheel_radius);

  stepper1.setSpeed(stepperSpeeds[0]);
  stepper2.setSpeed(stepperSpeeds[1]);
  stepper3.setSpeed(stepperSpeeds[2]);

  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  IMU.begin();
  myIMU.enableGyro(20); //Send data update every 20ms

  stepper1.setMaxSpeed(1000);	
  stepper2.setMaxSpeed(1000);	
  stepper3.setMaxSpeed(1000);	
}

void loop() {
  updateGyroStateVariables(&gyroRate);
  updateStepperSpeeds(&stateVector, &stepperSpeeds);
}
