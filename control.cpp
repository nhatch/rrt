#include <iostream>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>

#include <costmap_2d/costmap_2d.h>
#include "mppi/kinematic_mppi.h"
#include "mppi/stick_mppi.h"
#include "mppi/MPPILocalPlannerConfig.h"

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"
#include "control.h"

const double CONTROL_HZ = 30;
const double SPEED = 1.0;
const double MAX_DIFF = SPEED / CONTROL_HZ;

GraphNode *next_target; // sorry about the global var; will fix eventually TODO

bool getNextConfig(Config *current, const GraphNode *path, const Task &task,
    const graph_t &graph, KinematicMPPI& mppi, const costmap_2d::Costmap2D& costmap) {
  if (next_target == nullptr) {
    return true;
  }
  Config target = next_target->config;

  ControlArrayf u, v;
  StateArrayf x, g;
  x << current->x, current->y, current->theta;
  g << target.x, target.y, target.theta;
  mppi.setGoal(g);

  mppi.optimize(x, costmap);
  u = mppi.pop(x);
  u(2) /= pow(THETA_WEIGHT, 0.5);
  Config next = (*current) + Config(u(0), u(1), u(2));

  double d = current->distanceFrom(target);
  //double alpha = MAX_DIFF / d;
  if (d < MAX_DIFF) {
    //alpha = 1.0;
    next_target = next_target->parent;
  }
  //Config line = target - (*current);
  //Config next = (*current) + line * alpha;

  *current = next;
  return false;
}

void doControl(const GraphNode *path, const Task &task, const graph_t &graph) {
  Config current = path->config;
  next_target = path->parent;
  bool done = false;
  struct timeval tp0, tp1;
  double secsPerFrame = 1/CONTROL_HZ;

  MPPILocalPlannerConfig mppi_config;
  mppi_config.theta_weight = pow(THETA_WEIGHT, 0.5);
  StickMPPI mppi(mppi_config);
  Array3f goal;
  goal << 0., 0., 0.;

  mppi.reset();
  mppi.setGoal(goal);
  auto costmap = costmap_2d::Costmap2D(1000, 1000, 0.01, -2.5, -2.5);
  // Put a big obstacle in the middle
  for (unsigned int i = 300; i < 700; i++) {
    for (unsigned int j = 300; j < 700; j++) {
      costmap.setCost(i, j, 255);
    }
  }

  while (!done) {
    gettimeofday(&tp0, NULL);
    done = getNextConfig(&current, path, task, graph, mppi, costmap);
    drawStuff(current, task, graph);
    gettimeofday(&tp1, NULL);
    long elapsedUsecs = (tp1.tv_sec - tp0.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp0.tv_usec);
    long desiredUsecs = secsPerFrame * 1000 * 1000;
    if (desiredUsecs > elapsedUsecs)
      usleep(desiredUsecs - elapsedUsecs);
  }
}
