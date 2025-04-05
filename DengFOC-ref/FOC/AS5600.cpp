#include"AS5600.h"
#include <Wire.h>

/* RAW ANGLE REGISTER: 0x0C ---- 11:8 0x0D 7:0
 * STATUS 0x0B -- weak ML strong ---
 */

AS5600::AS5600(int8_t dir, bool zero){
  _DIR = dir;
  _ZEROANGLE = zero;
}

int8_t AS5600::checkStatus(){
  Wire.beginTransmission(I2C_ADD);
  Wire.write(STATUSREG);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADD, 1);
  while(Wire.available() == 0);

  STATUS = Wire.read();

  if(STATUS & (1<<5)){
    return 0;

  }
  else if(STATUS & (1<<4)){
    return 1;

  }
  else if(STATUS & (1<<3)){
    return 2;

  }
  else{
    return -1;

  }
}

void AS5600::initEncoder(){
  Wire.begin();
  Wire.setClock(400000L);
  int STATUS = checkStatus();
  
  if(STATUS == -1){
    Serial.println("Sensor not found.");

  }
  if(STATUS == 0){
    Serial.println("Magnetic field too weak.");

  }
  else if(STATUS == 2){
    Serial.println("Magnetic field too strong.");

  }

  STARTANGLE = _ZEROANGLE ? getRawAngle() : 0;
}

int AS5600::getRawAngle(){
  Wire.beginTransmission(I2C_ADD);
  Wire.write(HIGHREG);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADD, 2);
  while(Wire.available() < 2);

  HIGHBYTE = Wire.read();
  LOWBYTE = Wire.read();

  RAWANGLE = (HIGHBYTE << 8) | LOWBYTE;

  return RAWANGLE;
}

int AS5600::correctAngle(){
  OFFSETANGLE = RAWANGLE - STARTANGLE;
  return (OFFSETANGLE < 0) ? OFFSETANGLE + 4096 : OFFSETANGLE;
}

float AS5600::calcTotalAngle(){
  getRawAngle();

  int CORRECTED = correctAngle();

  // assign quadrant number based on angle
  QUAD = (CORRECTED >= 0 && CORRECTED <= 1024) ? 1 :
             (CORRECTED >= 3072 && CORRECTED <= 4096) ? 4 : 2;

  if(QUAD != PREVQUAD) {
    if(QUAD == 1 && PREVQUAD == 4) {
      TURNS ++;
    }
    else if(QUAD == 4 && PREVQUAD == 1) {
      TURNS --;
    }

    PREVQUAD = QUAD;
  }
  
  // converted to total radians 
  TOTALANGLE = TURNS * _2PI + CORRECTED * rawToRad; 

  return TOTALANGLE; 
}

float AS5600::getAbsoluteAngle(){
  getRawAngle();
  return correctAngle() * rawToRad;
}

float AS5600::getAbsoluteAngleDegrees(){
  getRawAngle();
  return correctAngle() * rawToDeg;
}

float AS5600::getAngle(){
  return calcTotalAngle();
}

float AS5600::getAngleDegrees(){
  return calcTotalAngle() * radToDeg;
}

