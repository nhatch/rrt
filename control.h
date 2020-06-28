#ifndef RRT_CONTROL_H
#define RRT_CONTROL_H

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"

const double CONTROL_HZ = 30;
const double SPEED = 1.0;
const double MAX_DIFF = SPEED / CONTROL_HZ;

const bool FULL_COSTMAP = false;
const bool NEAREST_NEIGHBOR = true;
const double MOTION_NOISE = 0.005;
const bool RENDER_CONFIG_SPACE = false;

using maps_t = std::vector<sf::Texture>;

void doControl(const GraphNode *path, const Task &task, const ArrayXXb& costmap, const graph_t &graph, const graph_t &min_graph, bool adaptive_carrot, bool deterministic);

#endif
