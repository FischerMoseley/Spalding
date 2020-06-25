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
#include <PID_v1.h>
#include <Wire.h>
#include "ICM_20948.h"
#include <math.h>

//using theta = pi/4
#define sec_theta 1.41421356237
#define csc_theta 1.41421356237
#define wheel_radius 5 //wheel radius in cm
#define ball_radius 12.13 //ball radius in cm

#define sqrt3 1.73205080757 

/*--------------- State Variables -----------------*/
struct AGMT{  double accX; 
              double accY; 
              double accZ; 
              double gyrX;
              double gyrY;
              double gyrZ;
              double magX;
              double magY;
              double magZ;  
              double temp;  };
AGMT sensorAGMT;

struct stateVector { double v_x; double v_y; double w_z; };
stateVector controlVector;

/*------------- Hardware Configuration ----------- */
//Stepper motor configuration
#define STEPS_PER_REV 200
#define MICROSTEPPING 16
#define STEPPER_MAX_SPEED 10000
AccelStepper stepper1(AccelStepper::DRIVER, 2, 5);
AccelStepper stepper2(AccelStepper::DRIVER, 3, 6);
AccelStepper stepper3(AccelStepper::DRIVER, 4, 7);

//IMU configuration
ICM_20948_I2C IMU;

struct PIDconfigContainer { 
  double yawRateSetpoint = 0;
  double pitchRateSetpoint = 0;
  double rollRateSetpoint = 0;
  double K_p = 1;
  double K_i = 0;
  double K_d = 0;
} PIDconfig;

//Specify links and initial tuning parameters
PID yawPID   (&sensorAGMT.gyrZ, &controlVector.w_z, &PIDconfig.yawRateSetpoint,   PIDconfig.K_p, PIDconfig.K_i, PIDconfig.K_d, DIRECT);
PID pitchPID (&sensorAGMT.gyrY, &controlVector.v_y, &PIDconfig.pitchRateSetpoint, PIDconfig.K_p, PIDconfig.K_i, PIDconfig.K_d, DIRECT);
PID rollPID  (&sensorAGMT.gyrX, &controlVector.v_x, &PIDconfig.rollRateSetpoint,  PIDconfig.K_p, PIDconfig.K_i, PIDconfig.K_d, DIRECT);


/*------------- Wrapper Functions ----------- */
void updateIMU(struct AGMT* newAGMT){
  if(IMU.dataReady()){
    IMU.getAGMT();
    newAGMT->accX = IMU.accX();
    newAGMT->accY = IMU.accY();
    newAGMT->accZ = IMU.accZ();

    newAGMT->gyrX = IMU.gyrX();
    newAGMT->gyrY = IMU.gyrY();
    newAGMT->gyrZ = IMU.gyrZ();

    newAGMT->magX = IMU.magX();
    newAGMT->magX = IMU.magX();
    newAGMT->magX = IMU.magX();

    newAGMT->temp = IMU.temp();
  }
}

void updateStepperSpeeds(struct stateVector controlVector){
  double x = controlVector.v_x;
  double y = controlVector.v_y; //negative to compensate for the fact that the IMU's axes are orientented differently
  double w_z = controlVector.w_z;

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

void printControlVector(struct stateVector controlVector){
  //Serial.print("ControlVector: v_x: ");
  Serial.print(controlVector.v_x);
  //Serial.print("  v_y: ");
  Serial.print(", ");
  Serial.print(controlVector.v_y);
  //Serial.print("  ω_Z: ");
  Serial.print(", ");
  Serial.print(controlVector.w_z);
  Serial.println();
}

void printGyroVector(struct AGMT currentAGMT){
  Serial.print("AGMT: gyr_x: ");
  Serial.print(currentAGMT.gyrX);
  Serial.print("  gyr_y: ");
  Serial.print(currentAGMT.gyrY);
  Serial.print("  gyr_z: ");
  Serial.print(currentAGMT.gyrZ);
  Serial.println();
}

void setup() {
  //start up all them interfaces
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  bool IMU_initialized = false;
  while (!IMU_initialized) {
    IMU.begin(Wire, 1);

    if (IMU.status != ICM_20948_Stat_Ok) {
      delay(500);
    }

    else {
      IMU_initialized = true;
    }
  }

  //set all the max motor speeds
  stepper1.setMaxSpeed(STEPPER_MAX_SPEED);	
  stepper2.setMaxSpeed(STEPPER_MAX_SPEED);	
  stepper3.setMaxSpeed(STEPPER_MAX_SPEED);

  //turn on PID
  yawPID.SetOutputLimits(-1000, 1000);
  pitchPID.SetOutputLimits(-1000, 1000);
  rollPID.SetOutputLimits(-1000, 1000);

  yawPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
}

void loop() {
  //get data
  updateIMU(&sensorAGMT);

  //recompute PID axis
  yawPID.Compute();
  pitchPID.Compute();
  rollPID.Compute();

  //update stepper speeds 
  updateStepperSpeeds(controlVector);
  printControlVector(controlVector);
  //printGyroVector(sensorAGMT);
  delay(2);
}