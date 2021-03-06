
#ifndef __GRAPH_H____
#define __GRAPH_H____

#include <vector>
#include "config.h"

constexpr bool MANUAL_GRAPH = false;
constexpr double ETA = MANUAL_GRAPH ? 10.0 : 0.1;
// Optimal path cost for 'gate' is something like 4.3, but
// for 'bugtrap' the planner sometimes returns paths of cost > 10.
// The disadvantage of large MAX_COST is decreased resolution for
// building the costmap when FULL_COSTMAP is true.
constexpr double MAX_COST = 100.0;

using obstacle_t = std::vector<point2d_t>;
struct projectile_t {
  Eigen::Array<float,1,2> location;
  Eigen::Array<float,1,2> velocity;

  projectile_t(float x, float y) : location(x,y), velocity(0,0) {
  }
};

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
