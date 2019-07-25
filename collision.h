#ifndef COLLISION_H
#define COLLISION_H

#include <vector>
#include "config.h"

using obstacle_t = std::vector<point2d_t>;
using task_t = std::vector<obstacle_t>;

bool collides(const Config &c, const task_t &task);
Config maxConfig(const Config &c0, const Config &c1, const task_t &task);

#endif
