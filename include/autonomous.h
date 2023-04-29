#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "vex.h"
#include "basic-functions.h"
#include "base.h"
#include "parameters.h"
#include "my-timer.h"
#include "GPS.h"

void auton_init(void);

void auton_pre_usercontrol(void);

void auton_sb(void);

///////////////////////////////////////////////////////
// your declarations of your autonomous here
///////////////////////////////////////////////////////

void auton_skill(void);

void auton_gps_test(void);

void runAuton(void);

#endif