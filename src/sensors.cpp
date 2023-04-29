#include "sensors.h"

Sensors my_sensors;

void Sensors::updateSensors() {
  base_left_pos = deg2rad(Motor_BaseLF.position(deg)+Motor_BaseLM.position(deg)) / 2 * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4) / 2.0 * BASE_FORWARD_COEFF;
  base_right_pos = deg2rad(Motor_BaseRF.position(deg)+Motor_BaseRM.position(deg)) / 2 * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4) / 2.0 * BASE_FORWARD_COEFF;
  base_left_vel = (Motor_BaseLF.velocity(pct) + Motor_BaseLM.velocity(pct)) / 2;
  base_right_vel = (Motor_BaseRF.velocity(pct) + Motor_BaseRM.velocity(pct)) / 2;
  base_heading = IMU.rotation() * 1800 / IMU_HEADING_5;
}

float Sensors::getBaseLeftPos() {
  return base_left_pos;
}

float Sensors::getBaseRightPos() {
  return base_right_pos;
}

float Sensors::getBaseForwardPos() {
  return (base_left_pos + base_right_pos) / 2;
}

float Sensors::getBaseLeftVel() {
  return base_left_vel;
}

float Sensors::getBaseRightVel() {
  return base_right_vel;
}

/**
 * return base forward velocity from -100 to 100
*/
float Sensors::getBaseForwardVel() {
  return (base_left_vel + base_right_vel) / 2;
}

float Sensors::getBaseHeading() {
  return base_heading;
}

void Sensors::setBaseHeading(float _heading) {
  IMU.setRotation(_heading, degrees);
}

void Sensors::resetBaseLeftPos() {
  Motor_BaseLF.resetPosition();
  Motor_BaseLM.resetPosition();
}

void Sensors::resetBaseRightPos() {
  Motor_BaseRF.resetPosition();
  Motor_BaseRM.resetPosition();
}

void Sensors::resetBasePos() {
  resetBaseLeftPos();
  resetBaseRightPos();
}

void Sensors::resetBaseHeading() {
  IMU.resetRotation();
}

void autonSensors() {
  while(1) {
    my_sensors.updateSensors();
    this_thread::sleep_for(5);
  }
}
