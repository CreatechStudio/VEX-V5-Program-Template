#ifndef GPS_H_
#define GPS_H_

#include "vex.h"
#include "string"

class GPS {
  private:
    float x_target, y_target, heading_target;
    float gps_x, gps_y, gps_heading;
    float length_delta, x_delta, y_delta;
    bool is_init;
    
    float calTheta(float , float, int=1);
    void calTargetPara(float, float, int=1);
    void setGpsXY(float, float);
    void setGpsHeading(float);
  public:
    GPS();
    void updateGpsPos();
    void initGPS(float=0, float=0, float=0);
    float getGpsX();
    float getGpsY();
    float getGpsHeading();

    void gpsPIDMove(float, float, int=1, int=90, std::string="PID");
    void gpsAim(float, float, float=0);
};

extern GPS my_gps;

void autonGPS(void);

#endif