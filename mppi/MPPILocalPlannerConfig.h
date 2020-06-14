
#ifndef _MPPI_CFG_H_
#define _MPPI_CFG_H_

class MPPILocalPlannerConfig {
  int mppi_rollouts = 1000;
  float time_horizon = 2.0;
  float lag = 0.0;
  float action_frequency = 50;
  int mppi_opt_iters = 1;
  float mppi_orientation_width = 5.;
  float mppi_covariance_prior_weight = 1.;
  int mppi_seed = 1;
  float mppi_collision_cost = 100.;
  float mppi_vel_wh_std = 0.2;
  float mppi_vel_x_std = 0.1;
  float mppi_vel_th_std = 0.3;
  float mppi_precision_superdiag = -1.0;
  bool mppi_optimize_wheel_speeds = true;
  float mppi_p_coeff = 10.f;
  float mppi_i_coeff = 1.f;
  float mppi_i_clamp = 10.f;
  float max_accel = -1.f;
  float max_vel_x = 1.f;
  float max_vel_th = 1.f;
  float speed_cost_threshold = 1.f;
  float speed_cost_weight = 0.f;
};

#endif
