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

double gyroPosition[3]; //literally have no idea how to get an euler output, just quaternions at the moment
double gyroRate[3]; //in the form x-axis rad/sec, y-axis rad/sec, z-axis rad/sec
double stateVector[3]; //in the form x-translation, y-translation, z-rotation

/*------------- Hardware Configuration ----------- */
//Stepper motor configuration
#define STEPS_PER_REV 200
#define STEPPER_MAX_SPEED 1000
AccelStepper stepper1(1, 3, 2);
AccelStepper stepper2(1, 5, 4);
AccelStepper stepper3(1, 6, 7);

//IMU configuration
BNO080 IMU;

//PID configuration
double xSetpoint = 0;
double ySetpoint = 0;

//define tuning parameters
double kP = 100;
double kI = 0;
double kD = 0;

//Specify links and initial tuning parameters
PID xAxisPID(&gyroRate[0], &stateVector[0], &xSetpoint, kP, kI, kD, DIRECT);
PID yAxisPID(&gyroRate[1], &stateVector[1], &ySetpoint, kP, kI, kD, DIRECT);


/*------------- Wrapper Functions ----------- */

void updateGyroStateVariables(double gyroRate[]){
  if(IMU.dataAvailable()){
    gyroRate[0] = IMU.getAccelX();
    gyroRate[1] = IMU.getAccelY();
    gyroRate[2] = IMU.getAccelZ();
  }
}

void updateStepperSpeeds(double stateVector[]){
  double stepperSpeeds[3];
  double x = stateVector[0];
  double y = stateVector[1];
  double w_z = stateVector[2];

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

void printStateVector(double stateVector[]){
  Serial.print("Gyro StateVector: X: ");
  Serial.print(stateVector[0]);
  Serial.print("  Y: ");
  Serial.print(stateVector[1]);
  Serial.print("  ω_Z: ");
  Serial.print(stateVector[2]);
  Serial.println();
}

void setup() {
  //start up all them interfaces
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  IMU.begin();
  IMU.enableAccelerometer(20); //Send data update every 20ms

  //set all the max motor speeds
  stepper1.setMaxSpeed(STEPPER_MAX_SPEED);	
  stepper2.setMaxSpeed(STEPPER_MAX_SPEED);	
  stepper3.setMaxSpeed(STEPPER_MAX_SPEED);

  //turn on PID
  xAxisPID.SetOutputLimits(-1000, 1000);
  yAxisPID.SetOutputLimits(-1000, 1000);
  xAxisPID.SetMode(AUTOMATIC);
  yAxisPID.SetMode(AUTOMATIC);
}

void loop() {
  //get data
  updateGyroStateVariables(gyroRate);

  //recompute PID axis
  xAxisPID.Compute();
  yAxisPID.Compute();

  printStateVector(stateVector);

  //update stepper speeds 
  updateStepperSpeeds(stateVector);
}