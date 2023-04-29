#ifndef PID_H_
#define PID_H_

#include "my-timer.h"

class PID {
private:
  float errorCrt, errorPrev, errorDev, errorInt;
  float P, I, D;
  bool firstTime;
  bool arrived;
  float kp, ki, kd;
  float target, errorTol, DTol;
  float IMax, IRange; // I < abs(IMAX) // I starts to increase when P < IRange
  float output;
  float jumpTime;
  MyTimer myTimer;

public:
  PID();
  void setFirstTime();
  void setCoefficient(float, float, float);
  void setTarget(float);
  void setIMax(float);
  void setIRange(float);
  void setErrorTolerance(float);
  void setDTolerance(float);
  void setJumpTime(float);
  void update(float input);
  bool targetArrived();
  float getOutput();
};


#endif