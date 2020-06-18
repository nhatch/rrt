#ifndef COLLISION_H
#define COLLISION_H

#include "config.h"
#include "rrt.h"

constexpr double COST_RESOLUTION_XY = 0.03;
constexpr double COST_RESOLUTION_TH = 0.12;
constexpr int COST_DIM_X = (MAX_X-MIN_X)/COST_RESOLUTION_XY;
constexpr int COST_DIM_Y = (MAX_Y-MIN_Y)/COST_RESOLUTION_XY;
constexpr int COST_DIM_TH = 2*M_PI/COST_RESOLUTION_TH;

bool collides(const Config &c, const Task &task, double clearance);
Config maxConfig(const Config &c0, const Config &c1, const Task &task, double clearance, bool *noCollision);

#endif
