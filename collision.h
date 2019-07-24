#ifndef COLLISION_H
#define COLLISION_H

#include <vector>
#include <array>

#include "config.h"

const size_t N_BALLS = 11;
const double LENGTH = 0.2;
const double BALL_RADIUS = LENGTH / (N_BALLS - 1);

using point2d_t = std::array<double,2>;
using obstacle_t = std::vector<point2d_t>;
using task_t = std::vector<obstacle_t>;
using balls_t = std::array<point2d_t,N_BALLS>;

balls_t getBalls(const Config &c);
bool collides(const Config &c, const task_t &task);
Config maxConfig(const Config &c0, const Config &c1, const task_t &task);

#endif
