#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>

const int I2C_ADD = 0x36;
const int STATUSREG = 0x0B;
const int HIGHREG = 0x0C;
const int LOWREG = 0x0D;

const float rawToRad = 0.00038349519f;
const float rawToDeg = 0.02197265625f;
const float _2PI = 6.28318530718f;
const float radToDeg = 57.2957795131f;


class AS5600{
  public:
  AS5600(int8_t dir, bool zero);

  void initEncoder();
  int getRawAngle();
  float getAbsoluteAngle();
  float getAbsoluteAngleDegrees();
  float getAngle();
  float getAngleDegrees();

  private:
  
  int checkStatus();
  int correctAngle();
  float calcTotalAngle();

  uint8_t STATUS;
  int HIGHBYTE;
  uint8_t LOWBYTE;
  int RAWANGLE;
  int STARTANGLE, OFFSETANGLE;
  uint8_t QUAD, PREVQUAD;
  int32_t TURNS;
  float TOTALANGLE;
  int8_t _DIR;
  bool _ZEROANGLE;

};

#endif