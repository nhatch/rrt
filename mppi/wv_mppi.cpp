#include "wv_mppi.h"

WheelVelocityMPPI::WheelVelocityMPPI(MPPILocalPlannerConfig &config)
  : KinematicMPPI(config) {
}

WheelVelocityMPPI::~WheelVelocityMPPI() {
}

void WheelVelocityMPPI::reset() {
  KinematicMPPI::reset();
  if (do_pi_tracking()) {
    pi_tracker_->reset();
  }
}

ArrayXXf WheelVelocityMPPI::toRigidBodyVels(ArrayXXf U) {
  assert(U.rows() % CONTROL_DIM == 0);
  ArrayXXf V;
  V.resizeLike(U);
  V.row(0) = (U.row(1) + U.row(0)) / 2.f; // linear velocity
  V.row(1) = (U.row(1) - U.row(0)) / TRACK; // rotational velocity
  return V;
}

void WheelVelocityMPPI::clamp(ArrayXXf& U) {
  Eigen::Map<ArrayXXf> U_flat(U.data(), CONTROL_DIM, rollouts_*sample_horizon_);
  U_flat = U_flat.max(-VEL_MAX).min(VEL_MAX);
  float max_deviation = max_vel_th_ * TRACK / 2.f;
  ArrayXXf mean_tiled = U_flat.colwise().mean().replicate<CONTROL_DIM,1>();
  U_flat = U_flat.max(mean_tiled - max_deviation).min(mean_tiled + max_deviation);
}
