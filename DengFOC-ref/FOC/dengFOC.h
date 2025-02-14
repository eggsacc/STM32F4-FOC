#ifndef DENGFOC
#define DENGFOC

#include <Arduino.h>

void FOCinit(uint8_t pole_pairs, uint8_t pwmA, uint8_t pwmB, uint8_t pwmC);
float velocityOpenLoop(float target_velocity);
float positionCloseLoop(float target_position);
void setPWM(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);

#endif