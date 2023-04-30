#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "vex.h"

const float MOVEMENT_LOWER_LIMIT = 5;
const float JOYSTICK_DEADZONE = 5;
const float CHASSIS_GEAR_RATIO = 0.75;
const float WHEEL_DIAMETER = 2.75;

#ifdef ROBOT_ONE
  const float BASE_FORWARD_PID[3] = {0, 0, 0};
  const float BASE_FORWARD_COEFF = 1;
  const float BASE_LEFT_POWER = 1.0;
  const float BASE_RIGHT_POWER = 1.0;
  const float BASE_ROTATE_BIG_PID[3] = {0, 0, 0};
  const float BASE_ROTATE_SMALL_PID[3] = {0, 0, 0};
  const float IMU_HEADING_5 = 1800;
#endif

#ifdef ROBOT_TWO
  const float BASE_FORWARD_PID[3] = {0, 0, 0};
  const float BASE_FORWARD_COEFF = 1;
  const float BASE_LEFT_POWER = 1.0;
  const float BASE_RIGHT_POWER = 1.0;
  const float BASE_ROTATE_BIG_PID[3] = {0, 0, 0};
  const float BASE_ROTATE_SMALL_PID[3] = {0, 0, 0};
  const float IMU_HEADING_5 = 1800;
#endif

#endif