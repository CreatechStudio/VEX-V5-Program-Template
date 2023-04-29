#ifndef ADJUSTMENT_H_
#define ADJUSTMENT_H_
#include "fstream"
#include "iostream"
#include "stdio.h"
#include "string"
#include "PID.h"

class Adjustment{
private:
    float input, output;
    std::ofstream my_ofstream;
    PID pid;
    MyTimer mytimer, globaltimer;
public:
    void updateInput(float);
    void updateOutput(float);
    void init(char*);
    void stop(void);
    void write(void);
    void startTimer(void);
    void stopTimer(void);
};
#endif

void tuning_forward_p(void);
void tuning_forward_i(void);
void tuning_forward_d(void);
void tuning_rotate_p(void);
void tuning_rotate_i(void);
void tuning_rotate_d(void);

void testing_forward(void);
void testing_rotate_big(void);
void testing_rotate_small(void);

void tuning_robot();