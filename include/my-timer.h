#ifndef MYTIMER_H_
#define MYTIMER_H_

#include "vex.h"
using namespace vex;

class MyTimer{
  private:
    float startTime;
  public:
    MyTimer();
    MyTimer(float);
    void reset();
    int getTime() const;
};

#endif