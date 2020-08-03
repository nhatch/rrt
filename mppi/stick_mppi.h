#ifndef _STICK_MPPI_H_
#define _STICK_MPPI_H_

#include "kinematic_mppi.h"
#include "mppi/MPPILocalPlannerConfig.h"

class StickMPPI : public KinematicMPPI {
  public:
    StickMPPI(MPPILocalPlannerConfig &config, const Task &task);

    ArrayXXf toRigidBodyVels(ArrayXXf U);
    void clamp(ArrayXXf& U);
};

#endif
