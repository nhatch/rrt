#ifndef RRT_CONTROL_H
#define RRT_CONTROL_H

#include "config.h"
#include "rrt.h"
#include "collision.h"

const double CONTROL_HZ = 30;
const double SPEED = 2.0;
const double MAX_DIFF = SPEED / CONTROL_HZ;
const bool MPPI_CHOOSE_OWN_GOAL = true;

void doControl(const GraphNode *path, const Task &task, const graph_t &graph, const graph_t &min_graph);

#endif
