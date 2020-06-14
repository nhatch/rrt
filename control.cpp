#include <iostream>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"
#include "control.h"

const double CONTROL_HZ = 30;
const double SPEED = 1.0;
const double MAX_DIFF = SPEED / CONTROL_HZ;

GraphNode *next_target; // sorry about the global var; will fix eventually TODO

bool getNextConfig(Config *current, const GraphNode *path, const Task &task, const graph_t &graph) {
  if (next_target == nullptr) {
    return true;
  }

  Config target = next_target->config;
  double d = current->distanceFrom(target);
  double alpha = MAX_DIFF / d;
  if (alpha > 1.0) {
    alpha = 1.0;
    next_target = next_target->parent;
  }
  Config line = target - (*current);
  Config next = (*current) + line * alpha;
  *current = next;
  return false;
}

void doControl(const GraphNode *path, const Task &task, const graph_t &graph) {
  Config current = path->config;
  next_target = path->parent;
  bool done = false;
  struct timeval tp0, tp1;
  double secsPerFrame = 1/CONTROL_HZ;
  while (!done) {
    gettimeofday(&tp0, NULL);
    done = getNextConfig(&current, path, task, graph);
    drawStuff(current, task, graph);
    gettimeofday(&tp1, NULL);
    long elapsedUsecs = (tp1.tv_sec - tp0.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp0.tv_usec);
    long desiredUsecs = secsPerFrame * 1000 * 1000;
    if (desiredUsecs > elapsedUsecs)
      usleep(desiredUsecs - elapsedUsecs);
  }
}
