#include "PID.h"
#include "math-tools.h"

PID::PID():firstTime(true),arrived(false),IMax(20),IRange(50),jumpTime(50) {
  myTimer.reset(); 
}

void PID::setFirstTime() {
  firstTime = true;
}

void PID::setCoefficient(float _kp, float _ki, float _kd) {
  kp = _kp;
  ki = _ki;
  kd = _kd;
}
void PID::setTarget(float _target) { target = _target;}
void PID::setIMax(float _IMax) { IMax = _IMax;}
void PID::setIRange(float _IRange) { IRange = _IRange;}
void PID::setErrorTolerance(float _errorTol) { errorTol = _errorTol;}
void PID::setDTolerance(float _DTol) { DTol = _DTol;}
void PID::setJumpTime(float _jumpTime) { jumpTime = _jumpTime;}
bool PID::targetArrived() { return arrived;}
float PID::getOutput() { return output;}

void PID::update(float input) {
  errorCrt = target - input; // calculate current error
  P = kp * errorCrt;
  if (firstTime) { // first time to update
    firstTime = false;
    errorPrev = errorCrt;
    errorInt = 0;
  }
  errorDev = errorCrt - errorPrev; // calculate the derivative of error
  errorPrev = errorCrt; // record error
  D = kd * errorDev; // calculate D
  if (fabs(P) >= IRange) { // I = 0 for P > IRange
    errorInt = 0;
  } 
  else { // P <= IRange -> Integrate
    errorInt += errorCrt;
    if (fabs(errorInt) * ki > IMax) // Limit I to IMax
      errorInt = sign(errorInt) * IMax / ki;
  }
  if (sign(errorInt) != sign(errorCrt) || (fabs(errorCrt) <= errorTol)) // Clear I for small enough error
    errorInt = 0;
  I = ki * errorInt; // Calculate I
  if (fabs(errorCrt) <= errorTol && fabs(D) <= DTol) { // Exit when staying in tolerated region and maintaining a low enough speed for enough time
    if (myTimer.getTime() >= jumpTime) arrived = true;
  } 
  else {
    myTimer.reset();
  }
  output = P + I + D;
  
}