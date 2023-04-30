#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

#ifdef ROBOT_ONE
  motor Motor_BaseLF = motor(PORT1, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT2, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT3, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT4, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT5, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT6, ratio6_1, false);
  inertial IMU = inertial(PORT9);
#endif

#ifdef ROBOT_TWO
  
#endif

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  IMU.startCalibration();
  while (IMU.isCalibrating()) {
    this_thread::sleep_for(5);
  }
  Controller1.Screen.setCursor(5, 1);
  Controller1.Screen.print("%19s", "CA");
  Controller1.Screen.setCursor(5, 1);
  Controller1.Screen.print("%10s", "l_6_2");
  
  // Brain.Screen.setCursor(12, 1);
  // Brain.Screen.print("Competition Version");
}