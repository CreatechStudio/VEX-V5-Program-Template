#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include "robot-config.h"
#include "vex.h"

// Define the buttons
int t, A1, A2, A3, A4, L1, L2, R1, R2, X, Y, A, B, LEFT, RIGHT, UP, DOWN,
    last_L1, last_L2, last_R1, last_R2, 
    last_X, last_Y, last_A, last_B, last_LEFT, last_RIGHT, last_UP, last_DOWN;
/**
 * @brief 更新手柄按键和摇杆的输入
 * 
 */
void defineController(){
    last_L1 = L1;
    last_L2 = L2;
    last_R1 = R1;
    last_R2 = R2;
    last_X = X;
    last_Y = Y;
    last_A = A;
    last_B = B;
    last_LEFT = LEFT;
    last_RIGHT = RIGHT;
    last_UP = UP;
    last_DOWN = DOWN;
    t = Brain.timer(vex::timeUnits::msec);
    A1 = Controller1.Axis1.position(vex::percentUnits::pct);
    A2 = Controller1.Axis2.position(vex::percentUnits::pct);
    A3 = Controller1.Axis3.position(vex::percentUnits::pct);
    A4 = Controller1.Axis4.position(vex::percentUnits::pct);
    L1 = Controller1.ButtonL1.pressing();
    L2 = Controller1.ButtonL2.pressing();
    R1 = Controller1.ButtonR1.pressing();
    R2 = Controller1.ButtonR2.pressing();
    X = Controller1.ButtonX.pressing();
    Y = Controller1.ButtonY.pressing();
    A = Controller1.ButtonA.pressing();
    B = Controller1.ButtonB.pressing();
    LEFT = Controller1.ButtonLeft.pressing();
    RIGHT = Controller1.ButtonRight.pressing();
    UP = Controller1.ButtonUp.pressing();
    DOWN = Controller1.ButtonDown.pressing();
}

#endif