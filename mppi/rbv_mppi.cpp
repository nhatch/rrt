#include "rbv_mppi.h"

RigidBodyVelocityMPPI::RigidBodyVelocityMPPI(MPPILocalPlannerConfig &config)
  : KinematicMPPI(config) {}

RigidBodyVelocityMPPI::~RigidBodyVelocityMPPI() {
}

ArrayXXf RigidBodyVelocityMPPI::toRigidBodyVels(ArrayXXf U) {
  assert(U.rows() % CONTROL_DIM == 0);
  return U;
}

void RigidBodyVelocityMPPI::clamp(ArrayXXf& U) {
  assert(U.rows() % CONTROL_DIM == 0);
  Eigen::Map<Array2Xf> U_flat(U.data(), CONTROL_DIM, U.size()/CONTROL_DIM);
  U_flat.row(0) = U_flat.row(0).max(-VEL_MAX).min(VEL_MAX);
  U_flat.row(1) = U_flat.row(1).max(-max_vel_th_).min(max_vel_th_);
}
