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
    const graph_t &graph, KinematicMPPI& mppi, const ArrayXXb& costmap) {
  Config target = next_stepwise_target->config;
  double d = current->distanceFrom(target);
  if (d < 3*MAX_DIFF) {
    if (next_stepwise_target->parent == nullptr) return true;
    if (!MPPI_CHOOSE_OWN_GOAL) next_stepwise_target = next_stepwise_target->parent;
  }

  ControlArrayf u, v;
  StateArrayf x, g;
  x << current->x, current->y, current->theta;

  if (!MPPI_CHOOSE_OWN_GOAL) {
    g << target.x, target.y, target.theta;
    mppi.setGoal(g);
  }

  mppi.optimize(x, costmap, graph);
  u = mppi.pop(x);
  u(2) /= pow(THETA_WEIGHT, 0.5);
  Config next = (*current) + Config(u(0), u(1), u(2));

  //double alpha = MAX_DIFF / d;
  //Config line = target - (*current);
  //Config next = (*current) + line * alpha;

  *current = next;
  return false;
}

void doControl(const GraphNode *path, const Task &task, const graph_t &graph, const graph_t &min_graph) {
  Config current = path->config;
  if (MPPI_CHOOSE_OWN_GOAL) {
    next_stepwise_target = graph[0];
  } else {
    next_stepwise_target = path->parent;
  }
  bool done = false;
  struct timeval tp0, tp1;
  double secsPerFrame = 1/CONTROL_HZ;

  MPPILocalPlannerConfig mppi_config;
  mppi_config.theta_weight = pow(THETA_WEIGHT, 0.5);
  StickMPPI mppi(mppi_config);
  Array3f goal;
  goal << 0., 0., 0.;

  std::cout << "Making costmap..." << std::flush;
  mppi.reset();
  mppi.setGoal(goal);
  ArrayXXb costmap;
  costmap.fill(false);
  costmap.resize(COST_DIM_X, COST_DIM_Y*COST_DIM_TH);
  for (unsigned int i = 0; i < COST_DIM_X; i++) {
    for (unsigned int j = 0; j < COST_DIM_Y; j++) {
      for (unsigned int k = 0; k < COST_DIM_TH; k++) {
        Config c(i*COST_RESOLUTION_XY + MIN_X,
                 j*COST_RESOLUTION_XY + MIN_Y,
                 k*COST_RESOLUTION_TH);
        if (collides(c, task)) {
          costmap(i, j*COST_DIM_TH + k) = true;
        }
      }
    }
  }
  std::cout << "done.\n";

  while (!done) {
    gettimeofday(&tp0, NULL);
    StateArrayf x;
    x << current.x, current.y, current.theta;
    StateArrayXf seq = mppi.rolloutNominalSeq(x);
    drawGraph(min_graph, task);
    Config goal(mppi.goal_(0), mppi.goal_(1), mppi.goal_(2));
    drawConfig(goal, sf::Color(0, 0, 255, 255));
    Config nearest(mppi.nearest_(0), mppi.nearest_(1), mppi.nearest_(2));
    drawConfig(nearest, sf::Color(255, 0, 0, 255));
    for (int i = 0; i < seq.cols(); i++) {
      if (i % 1 == 0) {
        StateArrayf x = seq.col(i);
        Config conf(x(0), x(1), x(2));
        if (i == 0) {
          drawConfig(conf, sf::Color(0, 255, 0, 255));
        } else {
          drawConfig(conf, sf::Color(128, 200, 128, 64));
        }
      }
    }
    doneDrawingStuff();

    done = getNextConfig(&current, path, task, graph, mppi, costmap);
    gettimeofday(&tp1, NULL);
    long elapsedUsecs = (tp1.tv_sec - tp0.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp0.tv_usec);
    long desiredUsecs = secsPerFrame * 1000 * 1000;
    if (desiredUsecs > elapsedUsecs)
      usleep(desiredUsecs - elapsedUsecs);
  }
}
