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

//using theta = pi/4
#define sec_theta 1.41421356237
#define csc_theta 1.41421356237
#define wheel_radius 5 //wheel radius in cm
#define ball_radius 12.13 //ball radius in cm

#define sqrt3 1.73205080757 

#define StepsPerRotation 200
#define StepperMaxSpeed 100

AF_Stepper motor1(StepsPerRotation, 1);
AF_Stepper motor2(StepsPerRotation, 1);
AF_Stepper motor3(StepsPerRotation, 1);

void forwardstep1() {motor1.onestep(FORWARD, SINGLE);}
void backwardstep1() {motor1.onestep(BACKWARD, SINGLE);}
void forwardstep2() {motor2.onestep(FORWARD, SINGLE);}
void backwardstep2() {motor2.onestep(BACKWARD, SINGLE);}
void forwardstep3() {motor3.onestep(FORWARD, SINGLE);}
void backwardstep3() {motor3.onestep(BACKWARD, SINGLE);}

AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);
AccelStepper stepper3(forwardstep3, backwardstep3);


float ConvertToAngular(float* new_vector, float* return_array){
  float x = new_vector[0];
  float y = new_vector[1];
  float w_z = new_vector[2];
  return_array[0] = (-2*x*csc_theta - w_z*ball_radius*sec_theta)/(3*wheel_radius);
  return_array[1] = ((x + sqrt3*y)*csc_theta - w_z*ball_radius*sec_theta)/(3*wheel_radius);
  return_array[2] = ((x - sqrt3*y)*csc_theta - w_z*ball_radius*sec_theta)/(3*wheel_radius);
}

void setup() {
  Serial.begin(115200);
  stepper1.setMaxSpeed(50);	
  stepper2.setMaxSpeed(50);	
  stepper3.setMaxSpeed(50);	
}

void loop() {
  stepper1.setSpeed(100);
  stepper2.setSpeed(100);
  stepper3.setSpeed(100);
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
}
