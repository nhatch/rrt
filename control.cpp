#include <iostream>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>

#include "mppi/kinematic_mppi.h"
#include "mppi/stick_mppi.h"
#include "mppi/MPPILocalPlannerConfig.h"

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"
#include "control.h"

GraphNode *next_stepwise_target;

bool getNextConfig(Config *current, const GraphNode *path, const Task &task,
    const graph_t &graph, KinematicMPPI& mppi, const ArrayXXb& costmap, bool adaptive_carrot, bool deterministic) {

  Config target = next_stepwise_target->config;
  double d = distanceFrom(*current, target);
  if (d < 3*MAX_DIFF) {
    if (next_stepwise_target->parent == nullptr) return true;
    if (deterministic || !adaptive_carrot) next_stepwise_target = next_stepwise_target->parent;
  }

  Config command;
  if (deterministic) {
    double alpha = MAX_DIFF / d;
    Config line = diff(target, (*current));
    command = line * alpha;
  }
  else
  {
    StateArrayf x = *current;

    if (!adaptive_carrot) {
      mppi.setGoal(target);
    }

    mppi.optimize(x, costmap, graph, adaptive_carrot);
    command = mppi.pop(x);
  }

  assert(distanceFrom(command, Config::Zero()) < MAX_DIFF*1.001);
  *current += command;
  return false;
}

void doControl(const GraphNode *path, const Task &task, const ArrayXXb& costmap, const graph_t &graph, const graph_t &min_graph, bool adaptive_carrot, bool deterministic) {
  Config current = path->config;
  if (!deterministic && (adaptive_carrot || FULL_COSTMAP)) {
    next_stepwise_target = graph[0];
  } else {
    next_stepwise_target = path->parent;
  }
  bool done = false;
  struct timeval tp0, tp1;
  double secsPerFrame = 1/CONTROL_HZ;

  MPPILocalPlannerConfig mppi_config;
  mppi_config.theta_weight = THETA_WEIGHT;
  mppi_config.mppi_seed = rand();
  StickMPPI mppi(mppi_config);
  Array3f goal;
  goal << 0., 0., 0.;
  mppi.reset();
  mppi.setGoal(goal);

  maps_t renders({});
  sf::Texture task_space_render = render(min_graph, task);
  if (RENDER_CONFIG_SPACE) {
    for (int i = 0; i < COST_DIM_TH; i++) {
      renders.push_back(render(costmap, i));
    }
  }

  double path_cost = 0.0;
  int n_steps = 0;
  while (!done) {
    gettimeofday(&tp0, NULL);
    StateArrayf x = current;
    StateArrayXf seq = mppi.rolloutNominalSeq(x);
    int theta_idx = floor(current(2) / COST_RESOLUTION_TH + 0.5);
    theta_idx = theta_idx % COST_DIM_TH;
    if (RENDER_CONFIG_SPACE) {
      drawTexture(renders[theta_idx]);
    } else {
      drawTexture(task_space_render);
    }
    if (!deterministic) {
      drawConfig(mppi.goal_, sf::Color(0, 0, 255, 255), RENDER_CONFIG_SPACE);
      drawConfig(mppi.nearest_, sf::Color(255, 0, 0, 255), RENDER_CONFIG_SPACE);
    }
    for (int i = 0; i < seq.cols(); i++) {
      if (i % 1 == 0) {
        StateArrayf x = seq.col(i);
        if (i == 0) {
          drawConfig(x, sf::Color(0, 255, 0, 255), RENDER_CONFIG_SPACE);
        } else {
          drawConfig(x, sf::Color(128, 200, 128, 64), RENDER_CONFIG_SPACE);
        }
      }
    }
    doneDrawingStuff();

    Config prev = current;
    done = getNextConfig(&current, path, task, graph, mppi, costmap, adaptive_carrot, deterministic);
    path_cost += distanceFrom(prev, current);
    n_steps += 1;

    Config noise = randConfig() * MOTION_NOISE;
    current = current + noise;
    if (collides(current, task, BALL_RADIUS/2.0)) {
      std::cout << "YOU DIED!!!!!!\n";
    }

    gettimeofday(&tp1, NULL);
    long elapsedUsecs = (tp1.tv_sec - tp0.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp0.tv_usec);
    long desiredUsecs = secsPerFrame * 1000 * 1000;
    if (desiredUsecs > elapsedUsecs)
      usleep(desiredUsecs - elapsedUsecs);
  }
  std::cout << path_cost << "," << n_steps << std::endl;
}
