#ifndef _RBV_MPPI_H_
#define _RBV_MPPI_H_

#include "kinematic_mppi.h"
#include "mppi/MPPILocalPlannerConfig.h"

/**
 * In this case, the controls are the rigid body velocities. That is,
 *   u = [linear velocity, angular velocity].
 */
class RigidBodyVelocityMPPI : public KinematicMPPI {
  public:
    RigidBodyVelocityMPPI(MPPILocalPlannerConfig &config);
    virtual ~RigidBodyVelocityMPPI();

  protected:
    ArrayXXf toRigidBodyVels(ArrayXXf U);
    void clamp(ArrayXXf& U);
};

#endif
