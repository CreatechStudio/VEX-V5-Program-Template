#include "math-tools.h"

int sign(float _input) {
  if (_input > 0) return 1;
  else if (_input < 0) return -1;
  else return 0;
}

float deg2rad(float deg) {
  return deg / 180.0 * M_PI;
}

float rad2deg(float rad) {
  return rad / M_PI * 180.0;
}

float calAbsDeltaAng(float _delta_ang) {
    while(fabs(_delta_ang) > 180) {
        if(_delta_ang > 0)
            _delta_ang -= 360;
        else 
            _delta_ang += 360;
    }
    return _delta_ang;
}

