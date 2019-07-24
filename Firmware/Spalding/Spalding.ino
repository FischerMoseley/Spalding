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
#include <math.h>

//using theta = pi/4
#define sec_theta 1.41421356237
#define csc_theta 1.41421356237
#define wheel_radius 5 //wheel radius in cm
#define ball_radius 12.13 //ball radius in cm

#define sqrt3 1.73205080757 

/*--------------- State Variables -----------------*/
struct eulerAngle{ double yaw; double pitch; double roll; };
eulerAngle rotationVector;
eulerAngle gyroRate;

struct stateVector { double v_x; double v_y; double w_z; };
stateVector controlVector;

/*------------- Hardware Configuration ----------- */
//Stepper motor configuration
#define STEPS_PER_REV 200
#define STEPPER_MAX_SPEED 10000
AccelStepper stepper1(1, 3, 2);
AccelStepper stepper2(1, 5, 4);
AccelStepper stepper3(1, 6, 7);

//IMU configuration
BNO080 IMU;

struct PIDconfigContainer { 
  double yawSetpoint = 0;
  double pitchSetpoint = 0;
  double rollSetpoint = 90;
  double K_p = 600;
  double K_i = 0;
  double K_d = 0;
} PIDconfig;

//Specify links and initial tuning parameters
//PID yawPID  (&rotationVector.yaw,   &controlVector.w_z, &PIDconfig.yawSetpoint,   PIDconfig.K_p, PIDconfig.K_i, PIDconfig.K_d, DIRECT);
PID pitchPID(&rotationVector.pitch, &controlVector.v_y, &PIDconfig.pitchSetpoint, PIDconfig.K_p, PIDconfig.K_i, PIDconfig.K_d, DIRECT);
PID rollPID (&rotationVector.roll,  &controlVector.v_x, &PIDconfig.rollSetpoint,  PIDconfig.K_p, PIDconfig.K_i, PIDconfig.K_d, DIRECT);


/*------------- Wrapper Functions ----------- */
void updateRotationVector(struct eulerAngle* rotationVector){
  if(IMU.dataAvailable()){
    float i = IMU.getQuatI();
    float j = IMU.getQuatJ();
    float k = IMU.getQuatK();
    float r = IMU.getQuatReal();
    float x; float y;

    /* YAW */ 
    x = 2 * ((i * j) + (r * k)); 
    y = square(r) - square(k) - square(j) + square(i); 
    rotationVector->yaw = degrees(atan2(y, x));

    /* PITCH */ 
    rotationVector->pitch = degrees(asin(-2 * (i * k - j * r)));

    /* ROLL */ 
    x = 2 * ((j * k) + (i * r)); 
    y = square(r) + square(k) - square(j) - square(i); 
    rotationVector->roll = degrees(atan2(y , x));
  }
}

void updateGyroRate(struct eulerAngle* angleVariable){
  if(IMU.dataAvailable()){
    angleVariable->yaw    =  IMU.getGyroX();
    angleVariable->pitch  =  IMU.getGyroY();
    angleVariable->roll   =  IMU.getGyroZ();
  }
}

void updateStepperSpeeds(struct stateVector* controlVector){
  double x = controlVector->v_x;
  double y = -1*controlVector->v_y; //negative to compensate for the fact that the IMU's axes are orientented differently
  double w_z = controlVector->w_z;

  double stepperSpeeds[3];
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

void printControlVector(struct stateVector* controlVector){
  Serial.print("ControlVector: v_x: ");
  Serial.print(controlVector->v_x);
  Serial.print("  v_y: ");
  Serial.print(controlVector->v_y);
  Serial.print("  ω_Z: ");
  Serial.print(controlVector->w_z);
  Serial.println();
}

void setup() {
  //start up all them interfaces
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  IMU.begin();
  IMU.enableRotationVector(10);

  //set all the max motor speeds
  stepper1.setMaxSpeed(STEPPER_MAX_SPEED);	
  stepper2.setMaxSpeed(STEPPER_MAX_SPEED);	
  stepper3.setMaxSpeed(STEPPER_MAX_SPEED);

  //turn on PID
  //yawPID.SetOutputLimits(-1000, 1000);
  pitchPID.SetOutputLimits(-1000, 1000);
  rollPID.SetOutputLimits(-1000, 1000);

  //yawPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
}

void loop() {
  //get data
  updateRotationVector(&rotationVector);

  //recompute PID axis
  //yawPID.Compute();
  pitchPID.Compute();
  rollPID.Compute();

  //update stepper speeds 
  updateStepperSpeeds(&controlVector);
}