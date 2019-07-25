#ifndef RRT_GRAPHICS_H
#define RRT_GRAPHICS_H

#include "config.h"
#include "rrt.h"
#include "collision.h"

void drawTree(const tree_t &tree, const task_t &task, bool render = true);
void animate(const Config &c0, const task_t &task);
void animate(const Config &c0, const Config &c1,
    const Config &start, const Config &end, const task_t &task, const tree_t *tree,
    double moveRate=0.005, double secsPerFrame=0.01);
void animatePath(const TreeNode *path, const Config &start, const Config &end, const task_t &task, const tree_t &tree);

#endif
