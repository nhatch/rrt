#ifndef RRT_CONTROL_H
#define RRT_CONTROL_H

#include "config.h"
#include "rrt.h"
#include "collision.h"

void doControl(const GraphNode *path, const Task &task, const graph_t &graph);

#endif
