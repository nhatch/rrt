#include "kinematic_mppi.h"
#include "../collision.h"
#include <Eigen/Eigenvalues>
#include <ros/console.h>

float KinematicMPPI::VEL_MAX = 1.f;
float KinematicMPPI::TRACK = 0.545f;
constexpr bool LEARN_COVARIANCE = false;
constexpr bool FULL_COVARIANCE = false;
const GraphNode *nominal_terminal_node = nullptr;

ArrayXf reshape(ArrayXXf array, size_t size) {
  assert(array.size() == size);
  return Eigen::Map<ArrayXf>(array.data(), size);
}

ArrayXXf reshape(ArrayXXf array, size_t rows, size_t cols) {
  assert(array.size() == rows*cols);
  return Eigen::Map<ArrayXXf>(array.data(), rows, cols);
}

KinematicMPPI::KinematicMPPI(MPPILocalPlannerConfig &config, const Task &task) : task_(task) {
  horizon_ = config.time_horizon;
  lag_horizon_ = config.lag;
  sample_horizon_ = horizon_ - lag_horizon_;
  rollouts_ = config.mppi_rollouts;
  temperature_ = config.mppi_temperature;
  orientation_width_ = config.mppi_orientation_width;
  alpha_ = config.mppi_covariance_prior_weight;
  collision_cost_ = config.mppi_collision_cost;
  opt_iters_ = config.mppi_opt_iters;
  speed_cost_threshold_ = VEL_MAX * config.speed_cost_threshold;
  speed_cost_weight_ = config.speed_cost_weight;
  nominal_cost_ = 1000.f; // Some big number
  theta_weight_ = config.theta_weight;

  ControlArrayf stds;
  stds << config.mppi_pos_std, config.mppi_pos_std, config.mppi_th_std;
  if (SECOND_ORDER) stds /= 3.0;
  if (FULL_COVARIANCE) {
    control_sqrt_cov_ = buildTridiag(stds, config.mppi_precision_superdiag);
    //control_sqrt_cov_wild_ = buildTridiag(0.2f, 0.2f, -5.f);
    //if (LEARN_COVARIANCE) control_sqrt_cov_prior_ = control_sqrt_cov_;
  } else {
    control_sqrt_cov_ = stds.replicate(sample_horizon_,1).matrix().asDiagonal();
  }

  U_.resize(Eigen::NoChange, sample_horizon_);
  control_history_.resize(Eigen::NoChange, lag_horizon_);
  Eigen::internal::scalar_normal_dist_op<float>::rng.seed(config.mppi_seed); // Seed the RNG
}

MatrixXf KinematicMPPI::buildTridiag(ControlArrayf &stds, float superdiag) {
  int CD = CONTROL_DIM;
  MatrixXf precision_sqrt(CD*sample_horizon_, CD*sample_horizon_);
  precision_sqrt.setZero();
  for (int i=0; i < horizon_; i++) {
    precision_sqrt.block(i*CD, i*CD, CD, CD) << 1.f/stds(0), 0, 0,
                                                0, 1.f/stds(1), 0,
                                                0, 0, 1.f/stds(2);
    if (i < horizon_-1)
      precision_sqrt.block(i*CD, (i+1)*CD, CD, CD) << superdiag, 0, 0,
                                                      0, superdiag, 0,
                                                      0, 0, superdiag;
  }
  return precision_sqrt.inverse();
}

KinematicMPPI::~KinematicMPPI() {}

void KinematicMPPI::setGoal(const Array3f& goal) {
  goal_ = goal;
}

void KinematicMPPI::reset() {
  U_.setZero();
  control_history_.setZero();
  if (LEARN_COVARIANCE)
    control_sqrt_cov_ = control_sqrt_cov_prior_;
  iters_since_reset_ = 0;
  avg_prediction_error_ = 0.f;
}

ControlArrayf KinematicMPPI::pop(const StateArrayf& state) {
  ControlArrayf u = U_.col(0);
  shift(state);
  return u;
}

void KinematicMPPI::shiftControlHistory() {
  if (lag_horizon_ <= 0)
    return;
  control_history_.leftCols(lag_horizon_-1) = control_history_.rightCols(lag_horizon_-1);
  control_history_.col(lag_horizon_-1) = U_.col(0);
}

void KinematicMPPI::shift(const StateArrayf& state) {
  shiftControlHistory();
  U_.leftCols(sample_horizon_-1) = U_.rightCols(sample_horizon_-1);
  U_.col(sample_horizon_-1).setZero();
}

void KinematicMPPI::optimize(const StateArrayf& state, const ArrayXXb& costmap, const graph_t& graph, bool adaptive_cost, const Task &task) {
  if (iters_since_reset_ > 0) {
    float prediction_error = (state - predicted_next_state_).matrix().norm();
    avg_prediction_error_ = (avg_prediction_error_ * (iters_since_reset_-1) + prediction_error) / iters_since_reset_;
    //ROS_WARN("Prediction error: %f", avg_prediction_error_);
  }
  iters_since_reset_ += 1;
  for (int i = 0; i < opt_iters_; i++) {
    SampledTrajs samples = sampleTrajs(state);
    recent_samples_ = samples; // for debugging
    nominal_traj_ = { rolloutNominalSeq(state), U_ };
    nominal_cost_ = computeCost(nominal_traj_, costmap, graph, adaptive_cost, task)(0);
    ArrayXf traj_costs = computeCost(samples, costmap, graph, adaptive_cost, task);
    updateControlSeq(samples.U_trajs, traj_costs);
  }
}

float KinematicMPPI::nominalCost() {
  return nominal_cost_;
}

SampledTrajs KinematicMPPI::sampleTrajs(const StateArrayf& state) {
  ArrayXXf U_trajs = sampleControlTrajs(state);
  ArrayXXf X_trajs = rolloutSamples(state, U_trajs);
  return { X_trajs, U_trajs };
}

ArrayXXf KinematicMPPI::sampleControlTrajs(const StateArrayf& state) {
  const int zero_rollouts = rollouts_ / 20;
  MatrixXf dU_norm_flat = MatrixXf::NullaryExpr(CONTROL_DIM*sample_horizon_, rollouts_, randN_);
  ArrayXXf U_trajs = (control_sqrt_cov_ * dU_norm_flat).array();
  ArrayXf U_flat = reshape(U_, CONTROL_DIM*sample_horizon_);
  U_trajs.leftCols(rollouts_ - zero_rollouts).colwise() += U_flat;
  U_trajs.rightCols(1) *= 0; // "panic button"
  if (SECOND_ORDER) {
    // I happen to know that the maximum speed is three times the maximum acceleration,
    // so we should always be able to stop within three timesteps.
    // We might be able to stop faster in some cases, but that would be more complicated
    // to implement.
    ControlArrayf stop = -state.bottomRows(3)/3;
    for (int i = rollouts_-zero_rollouts; i < rollouts_; i++) {
      for (int j = 0; j < 3; j++) {
        U_trajs.block(j*CONTROL_DIM, i, CONTROL_DIM, 1) += stop;
      }
    }
  }
  clamp(U_trajs);
  return U_trajs;
}


ArrayXXf KinematicMPPI::rolloutSamples(const StateArrayf& state, ArrayXXf& U_trajs) {
  assert(U_trajs.rows() == CONTROL_DIM*sample_horizon_);
  int rollouts = U_trajs.cols();

  ArrayXXf X_trajs(STATE_DIM*(horizon_+1), rollouts);
  StateArrayXf X_t, X_tp1;
  ControlArrayXf U_t;

  X_t = X_trajs.topRows(STATE_DIM).colwise() = state;
  for (int t = 0; t < horizon_; t++) {
    if (t < lag_horizon_) {
      U_t = control_history_.col(t).replicate(1,rollouts);
    } else {
      U_t = U_trajs.block(CONTROL_DIM*(t-lag_horizon_), 0, CONTROL_DIM, rollouts);
    }
    X_tp1 = step(X_t, U_t);
    X_t = X_trajs.block(STATE_DIM*(t+1), 0, STATE_DIM, rollouts) = X_tp1;
  }
  return X_trajs;
}

StateArrayXf KinematicMPPI::rolloutNominalSeq(const StateArrayf& state) {
  ArrayXXf U_trajs = reshape(U_, CONTROL_DIM*sample_horizon_, 1);
  ArrayXXf X_trajs = rolloutSamples(state, U_trajs);
  return reshape(X_trajs, STATE_DIM, horizon_+1);
}

StateArrayXf KinematicMPPI::step(const StateArrayXf& X, const ControlArrayXf& U) {
  ArrayXXf V;
  StateArrayXf X_next(STATE_DIM, X.cols());
  V = toRigidBodyVels(U);
  /*
  X_next.row(0) = X.row(0) + V.row(0)*cos(X.row(2))*dt_;
  X_next.row(1) = X.row(1) + V.row(0)*sin(X.row(2))*dt_;
  X_next.row(2) = X.row(2) + V.row(1)*dt_;
  */
  if (SECOND_ORDER) {
    X_next.row(3) = X.row(3) + V.row(0);
    X_next.row(4) = X.row(4) + V.row(1);
    X_next.row(5) = X.row(5) + V.row(2);
    X_next.row(0) = X.row(0) + X.row(3);
    X_next.row(1) = X.row(1) + X.row(4);
    X_next.row(2) = X.row(2) + X.row(5);

    // Copied from stick_mppi.cpp b/c passing a direct ref
    // to only part of an array was . . . impossible?
    ArrayXf norm_sq = X_next.row(3)*X_next.row(3) +
                      X_next.row(4)*X_next.row(4) +
                      THETA_WEIGHT*THETA_WEIGHT*X_next.row(5)*X_next.row(5);
    ArrayXf frac = pow(norm_sq, 0.5) / MAX_DIFF;
    frac = frac.max(1.0);
    X_next.row(3) /= frac;
    X_next.row(4) /= frac;
    X_next.row(5) /= frac;
  } else {
    X_next.row(0) = X.row(0) + V.row(0);
    X_next.row(1) = X.row(1) + V.row(1);
    X_next.row(2) = X.row(2) + V.row(2);
  }
  return X_next;
}

ArrayXf KinematicMPPI::computeCost(SampledTrajs& samples, const ArrayXXb &costmap, const graph_t& graph, bool adaptive_carrot, const Task &task) {
  int rollouts = samples.X_trajs.size() / STATE_DIM / (horizon_+1);
  ArrayXXf states = reshape(samples.X_trajs, STATE_DIM, (horizon_+1)*rollouts);

  ArrayXf terrain_flat_lethal;
  terrain_flat_lethal.resize(states.outerSize());
  terrain_flat_lethal.setZero();
  unsigned int mx, my;
  Eigen::Array<int,Eigen::Dynamic,Eigen::Dynamic> map_posns;
  map_posns.resize(3, states.outerSize());
  map_posns.row(0) = ((states.row(0)-MIN_X) / COST_RESOLUTION_XY).floor().cast<int>().max(0).min(COST_DIM_X-1);
  map_posns.row(1) = ((states.row(1)-MIN_Y) / COST_RESOLUTION_XY).floor().cast<int>().max(0).min(COST_DIM_Y-1);
  map_posns.row(2) = ((states.row(2))       / COST_RESOLUTION_TH + 0.5).floor().cast<int>();
  for (int i = 0; i < states.outerSize(); i++) {
    map_posns(2,i) = map_posns(2,i) % COST_DIM_TH;
    map_posns(2,i) = (map_posns(2,i) + COST_DIM_TH) % COST_DIM_TH; // Handle negative numbers, argh
  }
  map_posns.row(2) += COST_DIM_TH * map_posns.row(1);
  for (int i = 0; i < states.outerSize(); i++) {
    unsigned char raw_cost;
    raw_cost = costmap(map_posns(0,i), map_posns(2,i));
    if (raw_cost == 255) {
      terrain_flat_lethal(i) = 1000000.f;
    } else {
      terrain_flat_lethal(i) = (float)raw_cost / 255.f;
    }
  }

  // COMPUTE COLLISIONS WITH DYNAMIC OBSTACLES. dang this is a lot of work

  ArrayXf dynamic_lethal;
  dynamic_lethal.resize(states.outerSize());
  dynamic_lethal.setZero();

  ArrayXXf balls = states.topRows(2);
  ArrayXXf directions = balls;
  directions.row(0) = cos(states.row(2));
  directions.row(1) = sin(states.row(2));
  balls -= LENGTH * 0.5 * directions;
  double lethal_dist_thresh = 3*BALL_RADIUS + PROJECTILE_RADIUS;
  double stick_len = (N_BALLS-1)*BALL_RADIUS;
  double n_tests = 3;
  for (size_t i = 0; i < n_tests; i++) {
    for (projectile_t &p : task.projectiles) {
      Eigen::Array<float,2,1> eigp = p.location;
      if ((balls.col(0)-eigp).matrix().norm() < 2*lethal_dist_thresh) {
        ArrayXf distances = (balls.colwise() - eigp).matrix().colwise().norm().array();
        dynamic_lethal += ((lethal_dist_thresh - distances) / lethal_dist_thresh).max(0);
      }

    }
    balls += LENGTH/(n_tests-1) * directions;
  }
  dynamic_lethal *= 1000;

  if (adaptive_carrot) {
    double t_cost;
    StateArrayf x = nominal_traj_.X_trajs.block(0, horizon_, STATE_DIM, 1);
    const GraphNode *node = graph.nearestNode(x.topRows(3), task_, &t_cost);
    if (node != nullptr) {
      goal_ = node->config;
    }
  } else {
    nearest_ = goal_;
  }

  //ArrayXf speeds_flat = states.bottomRows(2).matrix().colwise().norm().array();
  //ArrayXf dist2goal_sq_flat = dist2goal_flat * dist2goal_flat;
  //ArrayXf speeds_sq_flat = speeds_flat * speeds_flat;
  //terrain_flat_inflated *= (speeds_flat - 0.5).max(0.0); // Penalize terrain more when going fast
  //ArrayXf speed_costs_coeffs_flat = 1.0 - dist2goal_flat.min(speed_cost_threshold_) / speed_cost_threshold_;
  //ArrayXf speed_costs_flat = speed_cost_weight_ * speeds_sq_flat * speed_costs_coeffs_flat;
  //ArrayXf yaw_costs_flat = -cos(yaws - goal_(2));
  // Subtract the minimum distance so that even when the robot is very far from the goal,
  // the distance cost will be roughly balanced with the obstacle/speed costs.
  //dist2goal_flat -= dist2goal_flat.minCoeff();
  ArrayXf costs_flat = collision_cost_*(terrain_flat_lethal + dynamic_lethal);
                       //+ dist2goal_flat;
                       //+ speed_costs_flat
                       //+ exp(-dist2goal_sq_flat/orientation_width_)*yaw_costs_flat

  ArrayXXf posn_errs = states.topRows(3).colwise() - goal_;
  posn_errs.row(2) *= theta_weight_;
  ArrayXf dist2goal_flat = posn_errs.matrix().colwise().norm().array();
  if (NEAREST_NEIGHBOR) {
    ArrayXf foo = dist2goal_flat - MAX_DIFF;
    ArrayXf binary_too_far_costs = 0.5 * foo / (foo.abs() + 0.00001) + 0.5;
    costs_flat += binary_too_far_costs;
  } else if (!FULL_COSTMAP) {
    costs_flat += dist2goal_flat;
  }

  ArrayXXf controls = reshape(samples.U_trajs, CONTROL_DIM, (horizon_)*rollouts);
  controls.row(2) *= THETA_WEIGHT;
  ArrayXf control_costs_flat = controls.matrix().colwise().norm().array();
  ArrayXXf costs = reshape(costs_flat, horizon_+1, rollouts);
  ArrayXXf control_costs = reshape(control_costs_flat, horizon_, rollouts);
  ArrayXf traj_costs = costs.colwise().sum() + control_costs.colwise().sum();

  if (NEAREST_NEIGHBOR) {
    for (int i = 0; i < rollouts; i++) {
      int terminal_state_idx = (i+1)*(horizon_+1) - 1;
      StateArrayf x = states.col(terminal_state_idx);
      double t_cost;
      const GraphNode *nearest = graph.nearestNode(x.topRows(3), task_, &t_cost);
      if (rollouts == 1) {
        nominal_terminal_node = nearest;
      }
      traj_costs(i) += t_cost/MAX_DIFF;
    }
  }

  return traj_costs;
}

void KinematicMPPI::updateControlSeq(const ArrayXXf& U_trajs, ArrayXf& traj_costs) {
  traj_costs -= traj_costs.minCoeff();
  ArrayXf weights = exp(-traj_costs/temperature_);
  weights /= weights.sum();

  MatrixXf U_new_flat = U_trajs.matrix() * weights.matrix();
  ControlArrayXf U_new = reshape(U_new_flat.array(), CONTROL_DIM, sample_horizon_);
  U_ = U_new.array();

  if (LEARN_COVARIANCE) {
    // U_new_flat only has one column
    MatrixXf centered = U_trajs.matrix().colwise() - U_new_flat.col(0);
    MatrixXf cov_new = centered * weights.matrix().asDiagonal() * centered.transpose();
    // For numerical stability
    cov_new += 0.00001f * MatrixXf::Identity(cov_new.rows(), cov_new.cols());
    Eigen::SelfAdjointEigenSolver<MatrixXf> es(cov_new);
    control_sqrt_cov_ = alpha_ * control_sqrt_cov_prior_ + (1.f-alpha_) * es.operatorSqrt();
  }
}

