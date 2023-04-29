#include "vex.h"
#include "basic-functions.h"
#include "base.h"
#include "my-timer.h"
#include "PID.h"
#include "parameters.h"
#include "iostream"
#include "fstream"
#include "math-tools.h"
using namespace vex;

Base my_base;

/**
 * move forward with _power for _duration msec
 * does not stop base when finishing
 * @param _power ranges from -100 : 100
 * @param _duration ms
*/
void Base::timerForward(float _power, int _duration) {
    moveForward(_power);
    this_thread::sleep_for(_duration);
    unlockBase();
}

/**
 * move forward with _power for _duration msec to _target_heading
 * does not stop base when finishing
 * @param _power ranges from -100 : 100
 * @param _duration ms
 * @param _target_heading abs deg
*/
void Base::timerForwardWithHeading(float _power, int _duration, float _target_heading) {
    auto my_timer = MyTimer();
    while (my_timer.getTime() < _duration) {
        float heading_error = _target_heading - getHeading();
        heading_error = calAbsDeltaAng(heading_error);
        float power_turn = heading_error * 1.0; // kp = 2.0
        if (fabs(power_turn) > 15) power_turn = sign(power_turn) * 15; // PLimit = 15
        moveLeft(_power + power_turn);
        moveRight(_power - power_turn);
        this_thread::sleep_for(5);
    }
    unlockBase();
}

/**
 * move forward with _power for _target displacement
 * does not stop base when finishing
 * use the sign of _power to determine direction
 * @param _power ranges from 0 : 100
 * @param _target real displacement
*/
void Base::posForwardRel(float _power, float _target_pos) {
    float start_pos = getForwardPos();
    float power = sign(_target_pos) * fabs(_power);
    moveForward(power);
    // std::cout << "forward" << std::endl;
    while (fabs(getForwardPos() - start_pos) < fabs(_target_pos)) {
        // std::cout << getForwardPos() << ", " << power << std::endl;
        this_thread::sleep_for(5);
    }
    unlockBase();
}

/**
 * move forward with _power to _target position
 * does not stop base when finishing
 * use the sign of (_target - input) to determine direction
 * @param _power ranges from 0 : 100
 * @param _target real displacement
*/
void Base::posForwardAbs(float _power, float _target_pos) {
    float target_rel = _target_pos - getForwardPos();
    posForwardRel(_power, target_rel);
}

/**
 * move forward with _power for _target_pos displacement while sticking to _target_heading angle
 * does not stop base when finishing
 * use the sign of _power to determine direction
 * @param _power ranges from -0 : 100
 * @param _target_pos real displacement
 * @param _target_heading abs deg
*/
void Base::posForwardRelWithHeading(float _power, float _target_pos, float _target_heading) {
    float start_pos = getForwardPos();
    float power = sign(_target_pos) * fabs(_power);
    // std::cout << "forward" << std::endl;
    while (fabs(getForwardPos() - start_pos) < fabs(_target_pos)) {
        float heading_error = _target_heading - getHeading();
        heading_error = calAbsDeltaAng(heading_error);
        float power_turn = heading_error * 1.0; // kp = 2.0
        if (fabs(power_turn) > 15) power_turn = sign(power_turn) * 15; // PLimit = 15
        moveLeft(power + power_turn);
        moveRight(power - power_turn);
        // std::cout << getForwardPos() << ", " << _power << std::endl;
        this_thread::sleep_for(5);
    }
    unlockBase();
}

/**
 * move forward with _power to _target position while sticking to _target_heading angle
 * does not stop base when finishing
 * use the sign of (_target - input) to determine direction
 * @param _power ranges from 0 : 100
 * @param _target_pos real displacement
 * @param _target_heading abs deg
*/
void Base::posForwardAbsWithHeading(float _power, float _target_pos, float _target_heading) {
    float targetPosRel = _target_pos - getForwardPos();
    Base::posForwardRelWithHeading(_power, targetPosRel, _target_heading);
}

/**
 * move forward for _target displacement
 * stops base when finishing
 * @param _target real displacement
*/
void Base::PIDPosForwardRel(float _target) {
    PIDPosForwardAbs(getForwardPos() + _target);
}

/**
 * move forward to _target position
 * stops base when finishing
 * @param _target real displacement
*/
void Base::PIDPosForwardAbs(float _target) {
    auto pid = PID();
    pid.setCoefficient(BASE_FORWARD_PID[0], BASE_FORWARD_PID[1], BASE_FORWARD_PID[2]);
    pid.setTarget(_target);
    pid.setIMax(20);
    pid.setIRange(5);
    pid.setErrorTolerance(1); // 设定误差小于多少的时候可以跳出循环
    pid.setDTolerance(5); // 设定速度小于多少的时候车可以跳出循环
    pid.setJumpTime(20);
    // std::cout << "pid-forward" << std::endl;
    while (!pid.targetArrived()) {
        pid.update(getForwardPos());
        moveForward(pid.getOutput());
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Forward Position: %4.1f", getForwardPos());
        // std::cout << getForwardPos() << ", " << pid.getOutput() << std::endl;
        this_thread::sleep_for(20);
    }
    unlockBase();
}

/** 
 * move forward to _target position with specific coeff
 * stops base when finishing
 * @param _target real displacement
*/
void Base::PIDPosForwardAbs(float _target, float _kp, float _ki, float _kd, float _tolerance) {
  // move forward to _target position
  // stops base when finishing
    auto pid = PID();
    pid.setCoefficient(_kp, _ki, _kd);
    pid.setTarget(_target);
    pid.setIMax(20);
    pid.setIRange(5);
    pid.setErrorTolerance(_tolerance);
    pid.setDTolerance(5);
    pid.setJumpTime(20);
    // std::cout << "pid-forward" << std::endl;
    while (!pid.targetArrived()){
        pid.update(getForwardPos());
        moveForward(pid.getOutput());
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Forward Position: %4.1f", getForwardPos());
        // std::cout << getForwardPos() << ", " << pid.getOutput() << std::endl;
        this_thread::sleep_for(20);
    }
    unlockBase();
}

/**
 * move forward with power gradually increase from _power_init to _power_final within _duration msec
 * does not stop base when finishing
 * @param _power_init ranges from -100 : 100
 * @param _power_final ranges from -100 : 100
 * @param _duration ms
*/
void Base::softStartTimerForward(float _power_init, float _power_final, int _duration) {
    auto my_timer = MyTimer();
    float step = (_power_final - _power_init) / _duration;
    // std::cout << "softstart-foarward" << std::endl;
    while (my_timer.getTime() < _duration) {
        moveForward(_power_init + my_timer.getTime() * step);
        // std::cout << getForwardPos() << ", " << _power_init + my_timer.getTime() * step << std::endl;
        this_thread::sleep_for(5);
    }
    unlockBase();
}

/**
 * rotate clockwise with _power for _duration msec
 * does not stop base when finishing
 * @param _power ranges from -100 : 100
 * @param _duration ms
*/
void Base::timerRotate(float _power, int _duration) {  
    moveClockwise(_power);
    this_thread::sleep_for(_duration);
    resetForwardPos();
    unlockBase();
}

/**
 * rotate clockwise with _power for _target angle
 * does not stop base when finishing
 * @param _power ranges from 0 : 100
 * @param _target deg that determine the rotation direction
*/
void Base::angleRotateRel(float _power, float _target) {
    float start_angle = getHeading();
    float power = sign(_target) * fabs(_power);
    moveClockwise(power);
    // std::cout << "rotate" << std::endl;
    while (fabs(getHeading() - start_angle) < fabs(_target)) {
        // std::cout << getHeading() << ", " << power << std::endl;
        this_thread::sleep_for(5);
    }
    resetForwardPos();
    unlockBase();
}

/**
 * rotate clockwise with _power to _target angle
 * does not stop base when finishing
 * @param _power ranges from 0 : 100
 * @param _target abs deg from -infinity to +infinity
*/
void Base::angleRotateAbs(float _power, float _target) {
    angleRotateRel(_power, _target - getHeading());
}

/**
 * rotate clockwise with _power for _target angle
 * @param _target rel deg (from - infinity to + infinity)
*/
void Base::PIDAngleRotateRel(float _target) {
    auto pid = PID();
    pid.setTarget(getHeading()+_target);
    pid.setIMax(20);
    pid.setIRange(20); //use if sentance to define the I coeff
    pid.setErrorTolerance(1);
    pid.setDTolerance(2);
    pid.setJumpTime(20);
    // std::cout << "pid-rotate" << std::endl;
    while (!pid.targetArrived()) {
        pid.setCoefficient(BASE_ROTATE_BIG_PID[0], BASE_ROTATE_BIG_PID[1], BASE_ROTATE_BIG_PID[2]);
        pid.update(getHeading());
        moveClockwise(pid.getOutput());
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Heading: %3.2f", getHeading());
        // std::cout << getHeading() << ", " << pid.getOutput() << std::endl;
        this_thread::sleep_for(20);
    }
    resetForwardPos();
    unlockBase();
}

/**
 * rotate clockwise with _power for _target angle
 * @param _target rel deg (from - infinity to + infinity)
*/
void Base::PIDAngleRotateRel(float _target, float _kp, float _ki, float _kd, float _tolerance) {
    auto pid = PID();
    pid.setTarget(getHeading()+_target);
    pid.setIMax(20);
    pid.setIRange(20); //use if sentance to define the I coeff
    pid.setErrorTolerance(_tolerance);
    pid.setDTolerance(2);
    pid.setJumpTime(20);
    // std::cout << "pid-rotate" << std::endl;
    while (!pid.targetArrived()) {
        pid.setCoefficient(_kp, _ki, _kd);
        pid.update(getHeading());
        moveClockwise(pid.getOutput());
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Heading: %3.2f", getHeading());
        // std::cout << getHeading() << ", " << pid.getOutput() << std::endl;
        this_thread::sleep_for(20);
    }
    resetForwardPos();
    unlockBase();
}

/**
 * rotate clockwise with _power to _target angle
 * stops base when finishing
 * @param _target abs deg (always rotate with angle smaller than 180)
*/
void Base::PIDAngleRotateAbs(float _target) {
    auto pid = PID();
    // data transfer to prevend from huge angle rotation
    while(fabs(_target-getHeading()) > 180) {
        if(_target-getHeading() > 0) {
            _target -= 360;
        }
        else {
            _target += 360;
        }
    }
    pid.setTarget(_target);
    pid.setIMax(20);
    pid.setIRange(20); //use if sentance to define the I coeff
    pid.setErrorTolerance(1);
    pid.setDTolerance(2);
    pid.setJumpTime(20);
    // std::cout << "pid-rotate" << std::endl;
    while (!pid.targetArrived()) {
        pid.setCoefficient(BASE_ROTATE_BIG_PID[0], BASE_ROTATE_BIG_PID[1], BASE_ROTATE_BIG_PID[2]);
        pid.update(getHeading());
        moveClockwise(pid.getOutput());
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Heading: %3.2f", getHeading());
        // std::cout << getHeading() << ", " << pid.getOutput() << std::endl;
        this_thread::sleep_for(20);
    }
    resetForwardPos();
    unlockBase();
}

void Base::PIDAngleRotateAbs(float _target, float _kp, float _ki, float _kd, float _tolerance) {
    auto pid = PID();
    // data transfer to prevend from huge angle rotation
    while(fabs(_target-getHeading()) > 180) {
        if(_target-getHeading() > 0) {
        _target -= 360;
        }
        else {
        _target += 360;
        }
    }
    pid.setCoefficient(_kp, _ki, _kd);
    pid.setTarget(_target);
    pid.setIMax(20);
    pid.setIRange(20);
    pid.setErrorTolerance(_tolerance);
    pid.setDTolerance(2);
    pid.setJumpTime(20);
    // std::cout << "pid-rotate" << std::endl;
    while (!pid.targetArrived()){
        pid.update(getHeading());
        moveClockwise(pid.getOutput());
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Heading: %3.2f", getHeading());
        // std::cout << getHeading() << ", " << pid.getOutput() << std::endl;
        this_thread::sleep_for(20);
    }
    resetForwardPos();
    unlockBase();
}

/**
 * rotate clockwise with power gradually increase from _power_init to _power_final within _duration msec
 * does not stop base when finishing
 * @param _power_init ranges from -100 : 100
 * @param _power_final ranges from -100 : 100
 * @param _duration ms
*/
void Base::softStartTimerRotate(float _power_init, float _power_final, int _duration) {
    auto my_timer = MyTimer();
    float step = (_power_final - _power_init) / _duration;
    // std::cout << "softstart-rotate" << std::endl;
    while (my_timer.getTime() < _duration) {
        moveClockwise(_power_init + my_timer.getTime() * step);
        // std::cout << getHeading() << ", " << _power_init + my_timer.getTime() * step << std::endl;
        this_thread::sleep_for(5);
    }
    resetForwardPos();
    unlockBase();
}

void Base::PIDPosCurveRel(float left_target, float right_target, float tolerance) {
  // move curved to _target position
  // stops base when finishing
    float _target = (left_target + right_target) / 2;
    float ratio = left_target / right_target;
    auto pid = PID();
    float k = 1;
    pid.setCoefficient(1.05, 0.05, 1.5);
    pid.setTarget(_target);
    pid.setIMax(30);
    pid.setIRange(20);
    pid.setErrorTolerance(tolerance);
    pid.setDTolerance(5);
    pid.setJumpTime(50);
    while (!pid.targetArrived()){
        float leftPos_err = (getForwardPos() / _target) * left_target - getLeftPos();
        float rightPos_err = (getForwardPos() / _target) * right_target - getRightPos();
        pid.update(getForwardPos());
        float PIDoutput = pid.getOutput();
        if (fabs(PIDoutput) > 90) PIDoutput = sign(PIDoutput) * 90;
        if (ratio > 1){
        moveLeft(PIDoutput + k * leftPos_err);
        moveRight(PIDoutput / ratio + k * rightPos_err);
        }
        else{
        moveLeft(pid.getOutput() * ratio + k * leftPos_err);
        moveRight(pid.getOutput() + k * rightPos_err);
        }
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Forward Position: %.1f                           ", getForwardPos());
        this_thread::sleep_for(5);
    }
    resetForwardPos();
    unlockBase();
}

void Base::PIDPosCurveAbs(float left_target, float right_target, float tolerance) {
    PIDPosCurveRel(getLeftPos() + left_target, getRightPos() + right_target, tolerance);
}