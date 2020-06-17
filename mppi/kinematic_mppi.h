#ifndef _KINEMATIC_MPPI_H_
#define _KINEMATIC_MPPI_H_

#include <eigen3/Eigen/Core>
#include "gaussian_rng.h"
#include "mppi/MPPILocalPlannerConfig.h"
#include "rrt.h"
#include "control.h"

/**
 * State representation:
 * x = [x-position, y-position, yaw, velocity 1, velocity 2]
 *
 * Control representation:
 * If the subclass is WheelVelocityMPPI,
 * u = [left wheel velocity, right wheel velocity],
 * each limited to the range [+/-VEL_MAX], with the difference bounded by max_vel_th_.
 * If the subclass is RigidBodyVelocityMPPI,
 * u = [longitudinal velocity, yaw rate],
 * with the first element limited to the range [+/-VEL_MAX] and the second element
 * limited to the range [+/-max_vel_th_].
 */

#define STATE_DIM 3
#define CONTROL_DIM 3

#define UNKNOWN_PENALTY 3 // somewhat arbitrary small cost

using Eigen::MatrixXf;
using Eigen::ArrayXXf;
using Eigen::ArrayXf;
using Eigen::Array3f;
typedef Eigen::Array<float, 2, Eigen::Dynamic> Array2Xf;

typedef Eigen::Array<float, STATE_DIM, 1> StateArrayf;
typedef Eigen::Array<float, CONTROL_DIM, 1> ControlArrayf;

typedef Eigen::Array<float, STATE_DIM, Eigen::Dynamic> StateArrayXf;
typedef Eigen::Array<float, CONTROL_DIM, Eigen::Dynamic> ControlArrayXf;

typedef struct SampledTrajs {
  ArrayXXf X_trajs; ///< sampled states, of size [STATE_DIM*(horizon_+1), rollouts_]
  ArrayXXf U_trajs; ///< sampled controls, of size [CONTROL_DIM*horizon_, rollouts_]
} SampledTrajs;

ArrayXf reshape(ArrayXXf array, size_t size);
ArrayXXf reshape(ArrayXXf array, size_t rows, size_t cols);

class KinematicMPPI {
  public:
    static float VEL_MAX; ///< maximum velocity of the robot
    static float TRACK; ///< distance between left and right sides of the robot

    KinematicMPPI(MPPILocalPlannerConfig &config);
    virtual ~KinematicMPPI();

    /**
     * @brief Sets the goal of the kinematic planner.
     * @param goal The goal pose of the robot.
     */
    void setGoal(const Array3f& goal);

    /**
     * @brief Resets the controller.
     */
    virtual void reset();

    /**
     * @brief Returns the first control and shifts the sequence one time step
     * forward.
     * @param state The current state.
     * @return The first control.
     */
    ControlArrayf pop(const StateArrayf& state);

    /**
     * @brief Shifts the nominal sequence one time step forward.
     * @param state The current state.
     */
    virtual void shift(const StateArrayf& state);

    /**
     * @brief Optimize the control sequence given the current state of the system.
     * @param state The current state of the system.
     * @param costmap The 2D map used to calculate trajectory costs.
     */
    void optimize(const StateArrayf& state, const ArrayXXb& costmap, const graph_t& graph);

    /**
     * @brief Rolls out nominal control sequence.
     * @param state The current state of the system.
     * @return The state trajectory induced by the nominal control sequence.
     */
    virtual StateArrayXf rolloutNominalSeq(const StateArrayf& state);

    float nominalCost();

    SampledTrajs recent_samples_; // for visualization / debugging
    SampledTrajs nominal_traj_; // for visualization / debugging
    Array3f goal_;
    Array3f nearest_;
  protected:
    // MPPI parameters
    int rollouts_;
    int horizon_;
    int lag_horizon_;
    int sample_horizon_;
    float dt_;
    int opt_iters_;
    float temperature_;
    float alpha_;
    MatrixXf control_sqrt_cov_prior_;
    MatrixXf control_sqrt_cov_;
    MatrixXf control_sqrt_cov_wild_;
    StateArrayf predicted_next_state_;
    float avg_prediction_error_;
    int iters_since_reset_;
    Eigen::internal::scalar_normal_dist_op<float> randN_; ///< Gaussian functor
    ControlArrayXf U_; ///< the nominal control sequence
    ControlArrayXf control_history_; // a short history of recent controls, to deal with time lag
    float nominal_cost_; // cost of the nominal control sequence from the most recent start state

    // Cost parameters
    float orientation_width_;
    float collision_cost_;
    float max_delta_v_;
    float max_vel_th_;
    float speed_cost_threshold_;
    float speed_cost_weight_;
    float theta_weight_;

    void shiftControlHistory();
    MatrixXf buildTridiag(float std1, float std2, float superdiag);

    /**
     * @brief Samples trajectories from the current distribution.
     * @param state The current state of the system.
     * @return The sampled trajectories
     */
    SampledTrajs sampleTrajs(const StateArrayf& state);

    /**
     * @brief Sample control trajectories from the current distribution.
     * @return The sampled control trajectories, of size [2*horizon_, rollouts_].
     */
    ArrayXXf sampleControlTrajs();

    /**
     * @brief Rolls out the control trajectories to produce state trajectories.
     * @param state The current state of the system.
     * @param U_trajs Control trajectories, of size [CONTROL_DIM*horizon_, rollouts_].
     * @return Rolled-out state trajectories, of size [STATE_DIM*(horizon_+1), rollouts_].
     */
    virtual ArrayXXf rolloutSamples(const StateArrayf& state, ArrayXXf& U_trajs);

    /**
     * @brief Steps through the kinematics.
     * @param X The batch of states, of size [STATE_DIM, cols].
     * @param U The batch of controls, of size [CONTROL_DIM, cols].
     * @return X_next The batch of next states, of size [STATE_DIM, cols].
     */
    StateArrayXf step(const StateArrayXf& X, const ControlArrayXf& U);

    /**
     * @brief Finds the costs of the given samples.
     * @param samples The sampled trajectories.
     * @param costmap The 2D map used to calculate trajectory costs.
     * @return The corresponding costs.
     */
    ArrayXf computeCost(SampledTrajs& samples, const ArrayXXb& costmap, const graph_t &graph);

    /**
     * @brief Updates the control sequence according to the MPPI update law.
     * @param U_trajs The sampled control sequences.
     * @param traj_costs The corresponding costs.
     */
    void updateControlSeq(const ArrayXXf& U_trajs, ArrayXf& traj_costs);

    /**
     * @brief Converts the control sequence to a sequence of rigid body velocities.
     * @param U The control sequence, of size [rows, cols].
     * @return The sequence of rigid body velocities, of size [rows, cols].
     */
    virtual ArrayXXf toRigidBodyVels(ArrayXXf U) = 0;

    /**
     * @brief Clamps the given controls to an allowable range.
     * @param U The control sequences, of size [rows, cols], to be clamped.
     */
    virtual void clamp(ArrayXXf& U) = 0;

};

#endif
