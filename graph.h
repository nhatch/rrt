
#ifndef __GRAPH_H____
#define __GRAPH_H____

#include <vector>
#include "config.h"

const bool MANUAL_GRAPH = false;
const double ETA = MANUAL_GRAPH ? 10.0 : 0.1;
const double MAX_COST = 10.0; // Optimal path cost is something like 4.3

using obstacle_t = std::vector<point2d_t>;
using projectile_t = point2d_t;

struct Task {
  const Config &start;
  const Config &end;
  const std::vector<obstacle_t> &obstacles;
  std::vector<projectile_t> &projectiles;
};

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

using nodelist_t = std::vector<GraphNode *>;

constexpr int G_X_LEN = 5;
constexpr int G_Y_LEN = 5;
constexpr int G_TH_LEN = 14;

class graph_t {
public:
  std::array<nodelist_t, G_X_LEN * G_Y_LEN * G_TH_LEN> buckets_;
  int num_nodes_;

  graph_t();
  GraphNode *nodeForConfig(const Config &c);
  const GraphNode *nearestNode(const Config &c, const Task &task, double *cost) const;
  void getBucketsAsList(const Config &c, std::vector<int> &result) const;
  void insert(GraphNode *node);
  int size() const;
};


#endif
