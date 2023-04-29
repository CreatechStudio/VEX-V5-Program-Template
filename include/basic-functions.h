#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "robot-config.h"

using namespace vex;

// Output functions
void moveLeft(float);
void moveLeftVel(float);
void lockLeft(void);
void unlockLeft(void);
void moveRight(float);
void moveRightVel(float);
void lockRight(void);
void unlockRight(void);
void moveForward(float);
void moveClockwise(float);
void lockBase(void);
void unlockBase(void);
///////////////////////////////////////////////////////
// declaration of your output functions here
///////////////////////////////////////////////////////

// Input functions
float getLeftPos();
float getRightPos();
float getForwardPos();
float getCrtVel();
void resetLeftPos();
void resetRightPos();
void resetForwardPos();
float getHeading();
///////////////////////////////////////////////////////
// declaration of your output functions here
///////////////////////////////////////////////////////

#endif