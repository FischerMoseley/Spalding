/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the i/j/k/real parts of the rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.
*/

#include <math.h>
#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;

// Get the euler angles 
struct Euler{ float yaw; float pitch; float roll; };
struct Quat{ float i; float j; float k; float real; };


Euler getAngles(Quat q, bool degrees);

Quat myQuat; 
Euler eul;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.begin();

  myIMU.enableRotationVector(10); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  //Serial.println(F("Output in form i, j, k, real, accuracy"));
  Serial.println(F("Output in form time(ms), yaw, pitch, roll, accuracy"));
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
    /*
    Serial.print(quatI, 2);
    Serial.print(F(","));
    Serial.print(quatJ, 2);
    Serial.print(F(","));
    Serial.print(quatK, 2);
    Serial.print(F(","));
    Serial.print(quatReal, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy, 2);
    Serial.print(F(","));
    */
    myQuat.i = quatI;
    myQuat.j = quatJ;
    myQuat.k = quatK;
    myQuat.real = quatReal;

    eul = getAngles(myQuat);
    Serial.print(eul.yaw, 2);
    Serial.print(F(","));
    Serial.print(eul.pitch, 2);
    Serial.print(F(","));
    Serial.print(eul.roll, 2);  
    Serial.println();
  }
}


// Return the Euler angle structure from a Quaternion structure 
Euler getAngles(Quat q){

  Euler ret_val;
  float x; float y;

  /* YAW */ 
  x = 2 * ((q.i * q.j) + (q.real * q.k)); 
  y = square(q.real) - square(q.k) - square(q.j) + square(q.i); 
  ret_val.yaw = degrees(atan2(y, x));

  /* PITCH */ 
  ret_val.pitch = degrees(asin(-2 * (q.i * q.k - q.j * q.real)));

  /* ROLL */ 
  x = 2 * ((q.j * q.k) + (q.i * q.real)); 
  y = square(q.real) + square(q.k) - square(q.j) - square(q.i); 
  ret_val.roll = degrees(atan2(y , x));

  return ret_val;

}