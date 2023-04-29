#ifndef BASE_H
#define BASE_H
#include "v5_vcs.h"
#include "robot-config.h"
#include "queue"
#include "math-tools.h"
#include "parameters.h"

class Base {
    private:
    bool is_aiming;
    public:
    void timerForward(float, int);
    void timerForwardWithHeading(float, int, float);
    void posForwardRel(float, float);
    void posForwardAbs(float, float);
    void posForwardRelWithHeading(float, float, float);
    void posForwardAbsWithHeading(float, float, float);
    void PIDPosForwardRel(float);
    void PIDPosForwardAbs(float);
    void PIDPosForwardAbs(float, float, float, float, float);
    void softStartTimerForward(float, float, int);

    void timerRotate(float, int);
    void angleRotateRel(float, float);
    void angleRotateAbs(float, float);
    void PIDAngleRotateRel(float);
    void PIDAngleRotateRel(float, float, float, float, float);
    void PIDAngleRotateAbs(float);
    void PIDAngleRotateAbs(float, float, float, float, float);
    void softStartTimerRotate(float, float, int);

    void PIDPosCurveRel(float, float, float=2);
    void PIDPosCurveAbs(float, float, float=2);

    void setAimingStatus(bool);
    int updateAutonAiming();
};

extern Base my_base;

void autonAiming(void);

#endif