#include "vex.h"
#include "base.h"
#include "PID.h"
#include "parameters.h"
#include "GPS.h"
#include "sensors.h"
#include "basic-functions.h"

using namespace vex;

GPS my_gps;

GPS::GPS() {
  x_target = 0;
  y_target = 0;
  heading_target = 0;
  gps_x = 0;
  gps_y = 0;
  gps_heading = 0;
  length_delta = 0;
  x_delta = 0;
  y_delta = 0;
  is_init = false;
}

/**
 * @return ang toward the goal from -180 to 180;
*/
float GPS::calTheta(float _dx, float _dy, int _direct){
  float theta = rad2deg(atan(_dx/_dy));
  if (_direct * _dy  < 0){
    if (_dy * _dx > 0) theta -= 180;
    else theta += 180;
  }
  return theta;
}

/**
 * calculate heading_target and length_delta. (update the x_target, y_target, x_delta and y_delta by the way)
*/
void GPS::calTargetPara(float _x_target, float _y_target, int _direct) {
  x_target = _x_target;
  y_target = _y_target;
  x_delta = x_target - gps_x;
  y_delta = y_target - gps_y;
  heading_target = calTheta(x_delta, y_delta, _direct);
  length_delta = sqrtf(powf(x_delta,2) + powf(y_delta,2));
}

void GPS::setGpsXY(float _x, float _y) {
  gps_x = _x;
  gps_y = _y;
  resetForwardPos();
}

void GPS::setGpsHeading(float _heading) {
  gps_heading = _heading;
  my_sensors.setBaseHeading(_heading);
}

/**
 * run loop in a thread to update the current base position and heading 
*/
void GPS::updateGpsPos(){
  static float last_forward__pos = my_sensors.getBaseForwardPos();
  static float last_heading = my_sensors.getBaseHeading();
  static float crt_forward_pos, crt_heading;

  if(!is_init) {
    last_forward__pos = my_sensors.getBaseForwardPos();
    last_heading = my_sensors.getBaseHeading();
    is_init = true;
  }
  crt_forward_pos = my_sensors.getBaseForwardPos();   
  crt_heading = my_sensors.getBaseHeading();

  gps_x += (crt_forward_pos - last_forward__pos) * sin(deg2rad(0.5 * (crt_heading + last_heading)));
  gps_y += (crt_forward_pos - last_forward__pos) * cos(deg2rad(0.5 * (crt_heading + last_heading)));
  gps_heading = crt_heading;

  last_forward__pos = crt_forward_pos;
  last_heading = crt_heading;
  Brain.Screen.setCursor(9, 1);
  Brain.Screen.print("GPS:  x:  %.1f   y:  %.1f", gps_x, gps_y);
}

/**
 * set the base position and heading before the auton start.
 * @param _x base x position
 * @param _y base y position
 * @param _heading base heading
*/
void GPS::initGPS(float _x, float _y, float _heading) {
  setGpsXY(_x, _y);
  setGpsHeading(_heading);
  is_init = false;
  int i = 0;
  while(my_sensors.getBaseHeading() != _heading && i < 10) {
    this_thread::sleep_for(5);
    i++;
  }
}

float GPS::getGpsX() {
  return gps_x;
}

float GPS::getGpsY() {
  return gps_y;
}

float GPS::getGpsHeading() {
  return gps_heading;
}
/**
 * move base toward target
 * @param _xGoal target x position
 * @param _yGoal target y position
 * @param _direct base direction +1 or -1 (+1: forward, -1: backward)
 * @param _maxPower max forward power
 * @param mode "continue" or "heading" (continue: move without ang rotate, heading: move without PID brake)
*/
void GPS::gpsPIDMove(float _x_target, float _y_target, int _direct, int _maxPower, std::string mode) {
  calTargetPara(_x_target, _y_target, _direct);
  if (mode != "continue") {
    my_base.PIDAngleRotateAbs(heading_target, BASE_ROTATE_BIG_PID[0], BASE_ROTATE_BIG_PID[1], BASE_ROTATE_BIG_PID[2], 2);
  }
  resetForwardPos();
  my_base.softStartTimerForward(my_sensors.getBaseForwardVel(), _maxPower*_direct, (_maxPower*_direct-my_sensors.getBaseForwardVel())*2);
  if (mode == "heading") {
    my_base.posForwardAbsWithHeading(_maxPower, length_delta*_direct, heading_target);
  }
  else {
    my_base.posForwardAbsWithHeading(_maxPower, 0.70*length_delta*_direct, heading_target);
    my_base.PIDPosForwardAbs(length_delta*_direct, BASE_FORWARD_PID[0], BASE_FORWARD_PID[1], BASE_FORWARD_PID[2], 2);
  }
}

/**
 * aim the target according to the gps position
 * @param _x_target x position of the target
 * @param _y_target y position of the target
*/
void GPS::gpsAim(float _x_target, float _y_target, float _heading_offset) {
  calTargetPara(_x_target, _y_target, -1);
  float delta_ang = calAbsDeltaAng(heading_target-my_sensors.getBaseHeading());
  if(fabs(delta_ang) < 15) {
    my_base.PIDAngleRotateAbs(heading_target+_heading_offset, BASE_ROTATE_SMALL_PID[0], BASE_ROTATE_SMALL_PID[1], BASE_ROTATE_SMALL_PID[2], 0.5);
  }
  else {
    // my_base.softStartTimerRotate(20, 100, 200);
    my_base.PIDAngleRotateAbs(heading_target+_heading_offset, BASE_ROTATE_BIG_PID[0], BASE_ROTATE_BIG_PID[1], BASE_ROTATE_BIG_PID[2], 0.5);
  }
}

void autonGPS() {
  // Thread Function, My GPS, use main to call
  while(1) {
    my_gps.updateGpsPos();
    this_thread::sleep_for(20);
  }
}