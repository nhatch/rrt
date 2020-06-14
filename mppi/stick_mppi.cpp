#include "stick_mppi.h"

StickMPPI::StickMPPI(MPPILocalPlannerConfig &config)
  : KinematicMPPI(config) {
}

ArrayXXf StickMPPI::toRigidBodyVels(ArrayXXf U) {
  assert(U.rows() % CONTROL_DIM == 0);
  return U;
}

void StickMPPI::clamp(ArrayXXf& U) {
  Eigen::Map<ArrayXXf> U_flat(U.data(), CONTROL_DIM, rollouts_*sample_horizon_);
  U_flat = U_flat.max(-VEL_MAX).min(VEL_MAX);
}
