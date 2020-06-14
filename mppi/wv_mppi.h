#ifndef _WV_MPPI_H_
#define _WV_MPPI_H_

#include "kinematic_mppi.h"
#include "mppi/MPPILocalPlannerConfig.h"

/**
 * In this case, the controls are the desired wheel velocities. That is,
 *   u = [desired left wheel velocity, desired right wheel velocity].
 */
class WheelVelocityMPPI : public KinematicMPPI {
  public:
    WheelVelocityMPPI(MPPILocalPlannerConfig &config);
    virtual ~WheelVelocityMPPI();

    ArrayXXf toRigidBodyVels(ArrayXXf U);
    void clamp(ArrayXXf& U);
};

#endif
