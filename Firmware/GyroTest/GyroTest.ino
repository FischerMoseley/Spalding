#include "ICM_20948.h"
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 

ICM_20948_I2C myICM;

void setup() {
    Serial.begin(115200);
    while(!Serial){};
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  
    bool initialized = false;
    while (!initialized) {
        myICM.begin( WIRE_PORT, AD0_VAL );

        Serial.print( "Initialization of the sensor returned: " );
        Serial.println( myICM.statusString() );

        if( myICM.status != ICM_20948_Stat_Ok ){
        Serial.println( "Trying again..." );
        delay(500);
        }
        else{
        initialized = true;
        }
    }
}

void loop() {
    if( myICM.dataReady() ){
        myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
        printScaledAGMT( myICM.agmt);   // This function takes into account the sclae settings from when the measurement was made to calculate the values with units
        delay(30);
    }
    else {
        Serial.println("Waiting for data");
        delay(500);
    }
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    Serial.print("-");
  }else{
    Serial.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      Serial.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    Serial.print(-val, decimals);
  }else{
    Serial.print(val, decimals);
  }
}

void printScaledAGMT( ICM_20948_AGMT_t agmt){
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( myICM.accX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( myICM.accY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( myICM.accZ(), 5, 2 );
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( myICM.gyrX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( myICM.gyrY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( myICM.gyrZ(), 5, 2 );
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat( myICM.magX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( myICM.magY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( myICM.magZ(), 5, 2 );
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat( myICM.temp(), 5, 2 );
  Serial.print(" ]");
  Serial.println();
}