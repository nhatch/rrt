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

bool getNextConfig(Config *current, const GraphNode *path, const Task &task,
    const graph_t &graph, KinematicMPPI& mppi, const costmap_2d::Costmap2D& costmap) {
  Config target = graph[0]->config;
  double d = current->distanceFrom(target);
  if (d < MAX_DIFF) {
    return true;
  }

  ControlArrayf u, v;
  StateArrayf x, g;
  x << current->x, current->y, current->theta;
  g << target.x, target.y, target.theta;
  mppi.setGoal(g);

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

void doControl(const GraphNode *path, const Task &task, const graph_t &graph) {
  Config current = path->config;
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
      //costmap.setCost(i, j, 255);
    }
  }

  while (!done) {
    gettimeofday(&tp0, NULL);
    StateArrayf x;
    x << current.x, current.y, current.theta;
    StateArrayXf seq = mppi.rolloutNominalSeq(x);
    drawGraph(graph, task);
    for (int i = 0; i < seq.cols(); i++) {
      if (i % 5 == 0) {
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
