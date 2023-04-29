#include "adjusment.h"
#include "basic-functions.h"
#include "base.h"
#include "sensors.h"

void Adjustment::updateInput(float _input) {
    input = _input;
}

void Adjustment::updateOutput(float _output) {
    output = _output;
}

void Adjustment::init(char* _str) {
    my_ofstream.open(_str);
}

void Adjustment::stop() {
    my_ofstream.close();
}

void Adjustment::write() {
    my_ofstream << input << ", " << output << std::endl;
}

void Adjustment::startTimer() {
    mytimer.reset();
}

void Adjustment::stopTimer() {
    std::cout << "autonomous time: " << mytimer.getTime()/1000.0 << " sec" << std::endl;
}

void tuning_forward_p() {
    Adjustment my_adjustment;
    PID my_pid;
    MyTimer my_timer;
    float kp = 0.2;
    float input, output;
    for(int i = 0; i<10; i++) {
        kp += 0.02;
        char filename[14];
        sprintf(filename, "forward-p-%1.2f.txt", kp);
        my_adjustment.init(filename);
        my_sensors.resetBasePos();
        my_pid.setTarget(600);
        my_pid.setCoefficient(kp, 0, 0);
        my_pid.setErrorTolerance(0.1);
        my_pid.setDTolerance(1);
        my_pid.setJumpTime(100);
        std::cout << kp << std::endl;
        my_timer.reset();
        // my_base.softStartTimerRotate(30, 100, 600);
        while(!my_pid.targetArrived() && my_timer.getTime()<2000) {
            input = my_sensors.getBaseForwardPos();
            output = my_pid.getOutput();
            my_pid.update(input);
            moveForward(output);
            my_adjustment.updateInput(input);
            my_adjustment.updateOutput(output);
            my_adjustment.write();
            this_thread::sleep_for(20);
        }
        my_adjustment.stop();
        my_base.PIDAngleRotateRel(180);
    }
}

void tuning_forward_i() {
    
}

void tuning_forward_d() {
    Adjustment my_adjustment;
    PID my_pid;
    MyTimer my_timer;
    float kp = 0.34, kd = 0.5;
    float input, output;
    for(int i = 0; i<25; i++) {
        kd += 0.02;
        char filename[18];
        sprintf(filename, "forward-d-%1.2f-%1.2f.txt", kp, kd);
        my_adjustment.init(filename);
        my_sensors.resetBasePos();
        my_pid.setTarget(600);
        my_pid.setCoefficient(kp, 0, kd);
        my_pid.setErrorTolerance(0.1);
        my_pid.setDTolerance(1);
        my_pid.setJumpTime(100);
        std::cout << kd << std::endl;
        my_timer.reset();
        // my_base.softStartTimerRotate(30, 100, 600);
        while(!my_pid.targetArrived() && my_timer.getTime()<2000) {
            input = my_sensors.getBaseForwardPos();
            output = my_pid.getOutput();
            my_pid.update(input);
            moveForward(output);
            my_adjustment.updateInput(input);
            my_adjustment.updateOutput(output);
            my_adjustment.write();
            this_thread::sleep_for(20);
        }
        my_adjustment.stop();
        my_base.PIDAngleRotateRel(180);
    }
}

void tuning_rotate_p() {
    Adjustment my_adjustment;
    PID my_pid;
    MyTimer my_timer;
    float kp = 0.8;
    float input, output;    
    for(int i = 0; i<21; i++) {
        kp += 0.05;
        char filename[13];
        sprintf(filename, "rotate-p-%1.2f.txt", kp);
        my_adjustment.init(filename);
        my_pid.setCoefficient(kp, 0, 0);
        my_pid.setTarget(180);
        my_pid.setErrorTolerance(0.1);
        my_pid.setDTolerance(1);
        my_pid.setJumpTime(100);
        std::cout << kp << std::endl;
        my_timer.reset();
        // my_base.softStartTimerRotate(30, 100, 600);
        while(!my_pid.targetArrived() && my_timer.getTime()<2500) {
            input = my_sensors.getBaseHeading();
            output = my_pid.getOutput();
            my_pid.update(input);
            moveClockwise(output);
            my_adjustment.updateInput(input);
            my_adjustment.updateOutput(output);
            my_adjustment.write();
            this_thread::sleep_for(20);
        }
        my_adjustment.stop();
        my_sensors.resetBaseHeading();
    }
}

void tuning_rotate_i() {

}

void tuning_rotate_d() {
    Adjustment my_adjustment;
    PID my_pid;
    MyTimer my_timer;
    float kp = 1.00, kd = 3.0;
    float input, output;    
    for(int i = 0; i<14; i++) {
        kd += 0.1;
        char filename[20];
        sprintf(filename, "rotate-d-%1.2f-%1.2f.txt", kp, kd);
        my_adjustment.init(filename);
        my_pid.setCoefficient(kp, 0, kd);
        my_pid.setTarget(180);
        my_pid.setErrorTolerance(0.1);
        my_pid.setDTolerance(1);
        my_pid.setJumpTime(100);
        std::cout << kd << std::endl;
        my_timer.reset();
        // my_base.softStartTimerRotate(30, 100, 600);
        while(!my_pid.targetArrived() && my_timer.getTime()<2500) {
            input = my_sensors.getBaseHeading();
            output = my_pid.getOutput();
            my_pid.update(input);
            moveClockwise(output);
            my_adjustment.updateInput(input);
            my_adjustment.updateOutput(output);
            my_adjustment.write();
            this_thread::sleep_for(20);
        }
        my_adjustment.stop();
        my_sensors.resetBaseHeading();
    }
}

void testing_forward(void) {
    Adjustment my_adjustment;
    int dir = 1;
    my_adjustment.startTimer();
    for(int i = 0; i<12; i++) {
        my_base.PIDPosForwardRel((600-50*i)*dir);
        dir*=-1;
    }
    my_adjustment.stopTimer();
}

void testing_rotate_big(void) {
    Adjustment my_adjustment;
    my_adjustment.startTimer();
    for(int i = 0; i<12; i++) {
        // my_base.softStartTimerRotate(20, 100, 100);
        my_base.PIDAngleRotateRel(180-15*i, BASE_ROTATE_BIG_PID[0], BASE_ROTATE_BIG_PID[1], BASE_ROTATE_BIG_PID[2], 1);
    }
    my_adjustment.stopTimer();
}

void testing_rotate_small(void) {
    Adjustment my_adjustment;
    my_adjustment.startTimer();
    for(int i = 1; i<15; i++) {
        my_base.PIDAngleRotateRel(1*i, BASE_ROTATE_SMALL_PID[0], BASE_ROTATE_SMALL_PID[1], BASE_ROTATE_SMALL_PID[2], 1);
    }
    my_adjustment.stopTimer();
}

void tuning_robot() {
    // tuning_forward_p();
    // tuning_forward_i();
    // tuning_forward_d();
    // tuning_rotate_p();
    // tuning_rotate_i();
    // tuning_rotate_d();
    // testing_forward();
    // testing_rotate_big();
    // testing_rotate_small();
}