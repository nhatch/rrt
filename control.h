#ifndef RRT_CONTROL_H
#define RRT_CONTROL_H

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"

const double CONTROL_HZ = 30;
const double SPEED = 1.0;
const double MAX_DIFF = SPEED / CONTROL_HZ;

const bool FULL_COSTMAP = true;

void doControl(const GraphNode *path, const Task &task, const ArrayXXb& costmap, const sf::Texture &rendered_costmap, const graph_t &graph, const graph_t &min_graph, bool adaptive_carrot, bool deterministic);

#endif
