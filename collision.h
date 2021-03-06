#ifndef COLLISION_H
#define COLLISION_H

#include "config.h"
#include "graph.h"

constexpr int COST_DIM_X = 128;
constexpr int COST_DIM_Y = 128;
constexpr int COST_DIM_TH = 128;
constexpr double COST_RESOLUTION_XY = (MAX_X-MIN_X)/COST_DIM_X;
constexpr double COST_RESOLUTION_TH = 2*M_PI / COST_DIM_TH;

bool collides(const Config &c, const Task &task, double clearance);
Config maxConfig(const Config &c0, const Config &c1, const Task &task, double clearance, bool *noCollision);

#endif
