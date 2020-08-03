#ifndef RRT_H
#define RRT_H

#include <vector>
#include "config.h"

const bool MANUAL_GRAPH = false;
const double ETA = MANUAL_GRAPH ? 10.0 : 0.1;

struct GraphNode {
  const Config config;
  // Directed graph (though I anticipate for this example I'll
  // always have edges going both directions).
  std::vector<GraphNode *> children;
  // costs[i] is the cost of the edge going to children[i]
  std::vector<double> costs;
  GraphNode *parent;
  double cost;
};

using graph_t = std::vector<GraphNode *>;

using obstacle_t = std::vector<point2d_t>;

struct Task {
  const Config &start;
  const Config &end;
  const std::vector<obstacle_t> &obstacles;
};

GraphNode *nearestNode(const graph_t &graph, const Config &x, const Task &task, double *cost);

#endif
