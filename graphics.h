#ifndef RRT_GRAPHICS_H
#define RRT_GRAPHICS_H

#include "config.h"
#include "rrt.h"
#include "collision.h"

void drawGraph(const graph_t &graph, const Task &task);
void drawStuff(const Config &config, const Task &task, const graph_t &graph);
void animatePath(const GraphNode *path, const Task &task, const graph_t &graph,
    double movementPerFrame=0.01, double secsPerFrame=0.01);

#endif
