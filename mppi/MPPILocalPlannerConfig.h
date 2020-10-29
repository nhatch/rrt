
#ifndef _MPPI_CFG_H_
#define _MPPI_CFG_H_

struct MPPILocalPlannerConfig {
  int mppi_rollouts = 100;
  int time_horizon = 15;
  int lag = 0;
  int mppi_opt_iters = 1;
  float mppi_orientation_width = 5.;
  float mppi_covariance_prior_weight = 1.;
  int mppi_seed = 1;
  float mppi_collision_cost = 100.;
  float mppi_pos_std = 0.005;
  float mppi_th_std = 0.015;
  float mppi_precision_superdiag = -1.0;
  bool mppi_optimize_wheel_speeds = true;
  float mppi_p_coeff = 10.f;
  float mppi_i_coeff = 1.f;
  float mppi_i_clamp = 10.f;
  float speed_cost_threshold = 1.f;
  float speed_cost_weight = 0.f;
  float theta_weight = 1.f;
  float mppi_temperature = 1.5f; // This seems to avoid getting stuck and also avoid collisions
};

#endif
