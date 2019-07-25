#ifndef RRT_GRAPHICS_H
#define RRT_GRAPHICS_H

#include "config.h"
#include "rrt.h"
#include "collision.h"

void drawTree(const tree_t &tree, const Task &task);
void animatePath(const TreeNode *path, const Task &task, const tree_t &tree,
    double movementPerFrame=0.01, double secsPerFrame=0.01);

#endif
