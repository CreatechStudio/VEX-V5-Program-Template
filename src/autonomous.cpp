#include "autonomous.h"
#include "iostream"

static auto auton_timer = MyTimer();

/**
 * initialize the timer & gps and so on
*/
void auton_init(void) {
  // init
  my_gps.initGPS();
  // your initialization here
  auton_timer.reset();
}


/**
 * initialize pre user control and stop the timer
*/
void auton_pre_usercontrol(void) {
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("AutonTimer: %2.2fsec", auton_timer.getTime()/1000.0);
  // your pre user control here
}

/**
 * this auton program do nothing
*/
void auton_sb(void) {
  auton_init();
  auton_pre_usercontrol();
}

///////////////////////////////////////////////////////
// Your Autonomous here
// use void left/right_<logcal max score>_<pre load>(void){}
// don't forget to declare in the autonomous.h file
// with void left/right_<logcal max score>_<pre load>(void);
///////////////////////////////////////////////////////

/**
 * this auton program do skill programing challenge
*/
void auton_skill(void){
  auton_init();
  // skill challenge auton code here
  auton_pre_usercontrol();
}

/**
 * this auton program will run a 'z' route to test the gps
*/
void auton_gps_test(void) {
  auton_init();
  my_gps.gpsPIDMove(600, 0, 1, 60);
  my_gps.gpsPIDMove(0, 600, 1, 60);
  my_gps.gpsPIDMove(600, 600, 1, 60);
  my_gps.gpsPIDMove(0, 0, 1, 60);
  my_gps.gpsAim(0, -1200, 0);
  auton_pre_usercontrol();
}

/**
 * this function is used to trigger the autonomous manually without a field control with 'DOWN' key
*/
void runAuton(void) {
  // auton_skill();
  // auton_gps_test();
}