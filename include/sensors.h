#ifndef SENSORS_H
#define SENSORS_H
#include "v5_vcs.h"
#include "robot-config.h"
#include "queue"
#include "math-tools.h"
#include "parameters.h"

class Sensors {
  private:
  float base_left_pos, base_right_pos, base_left_vel, base_right_vel;
  float base_heading;
  public:
  Sensors() {};
  void updateSensors(void);
  float getBaseLeftPos(void);
  float getBaseRightPos(void);
  float getBaseForwardPos(void);
  float getBaseLeftVel(void);
  float getBaseRightVel(void);
  float getBaseForwardVel(void);
  float getBaseHeading(void);
  void setBaseHeading(float);
  void resetBaseLeftPos(void);
  void resetBaseRightPos(void);
  void resetBasePos(void);
  void resetBaseHeading(void);
};

extern Sensors my_sensors;

void autonSensors(void);

#endif 