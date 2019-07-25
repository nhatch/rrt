#ifndef COLLISION_H
#define COLLISION_H

#include "config.h"
#include "rrt.h"

bool collides(const Config &c, const Task &task);
Config maxConfig(const Config &c0, const Config &c1, const Task &task);

#endif
