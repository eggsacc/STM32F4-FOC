#include "dengFOC.h"
#include "FOCutils.h"
#include <MT6701.h>
#include <Arduino.h>
#include "AS5600.h"

int voltage_limit;
int voltage_power_supply;
    
float shaft_angle;
float angle_el;
float angle;
float Ua, Ub, Uc, Uq, Ud, Ualpha, Ubeta;
float dc_a, dc_b, dc_c;
float zero_electric_angle;
    
uint8_t pwmA;
uint8_t pwmB;
uint8_t pwmC;
uint8_t pp;

// Init ESP32 pwm chanels, 30kHz
void FOCinit(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t pole_pairs){
  
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  ledcAttach(pinA, 30000, 8);
  ledcAttach(pinB, 30000, 8);
  ledcAttach(pinC, 30000, 8);

  pwmA = pinA;
  pwmB = pinB;
  pwmC = pinC;
  pp = pole_pairs;
}

// microcontroller PWM output
void setPWM(float Ua, float Ub, float Uc){

  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  ledcWrite(pwmA, dc_a * 255);
  ledcWrite(pwmB, dc_b * 255);
  ledcWrite(pwmC, dc_c * 255);
}

// Inverse park, clarke transform
void setPhaseVoltage(float Uq, float Ud, float _angle_el) {
  angle_el = _normalizeAngle(_angle_el + zero_electric_angle);

  // inverse park transformation
  Ualpha = -Uq * _sin(angle_el);
  Ubeta = Uq * _cos(angle_el);

  // inverse clarke transformation
  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (_SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = (- Ualpha - _SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;

  setPWM(Ua, Ub, Uc);
}

// ************************************** Open loop velocity **************************************

// check time elapsed and generate new angle value
float setVelocity(float target_velocity) {
  unsigned long now_us = micros(); // in microseconds

  float Ts = (now_us - open_loop_timestamp) * 1e-6f; 

  if(Ts <= 0 || Ts > 0.5f){
    Ts = 1e-3f;
  }

  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts); 
  float Uq = voltage_limit;
  setPhaseVoltage(Uq, 0, electricalAngle(shaft_angle)); 

  open_loop_timestamp = now_us;

  return Uq;
}

// ************************************** Serial commander **************************************

void serialCommander(){

  String command = receiveCommand();

  space_index = command.indexof(" ");

  if(space_index != -1){
    String part = command.substring(0, space_index);
    String valPart = command.substring(space_index + 1);
    float val = valPart.toFloat();

    
  }

}

String receiveCommand(){
  String cmd = "";
  static String cmd_buffer;

  while(Serial.available()){
    char inChar = (char)Serial.read();
    cmd_buffer += inChar;
    if(inChar == '\n'){
      cmd = cmd_buffer;
    }
  }

  cmd_buffer = "";

  return cmd;
}