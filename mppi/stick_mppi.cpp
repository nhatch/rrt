#include "stick_mppi.h"
#include "../control.h"

StickMPPI::StickMPPI(MPPILocalPlannerConfig &config, const Task &task)
  : KinematicMPPI(config, task) {
}

ArrayXXf StickMPPI::toRigidBodyVels(ArrayXXf U) {
  assert(U.rows() % CONTROL_DIM == 0);
  return U;
}

void StickMPPI::clamp(ArrayXXf& U) {
  Eigen::Map<ArrayXXf> U_flat(U.data(), CONTROL_DIM, rollouts_*sample_horizon_);
  ArrayXf norm_sq = U_flat.row(0)*U_flat.row(0) +
                    U_flat.row(1)*U_flat.row(1) +
                    THETA_WEIGHT*THETA_WEIGHT*U_flat.row(2)*U_flat.row(2);
  ArrayXf frac = pow(norm_sq, 0.5) / MAX_COMMAND;
  //frac = frac.max(1.0);
  U_flat.row(0) /= frac;
  U_flat.row(1) /= frac;
  U_flat.row(2) /= frac;
}
