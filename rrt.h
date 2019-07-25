#ifndef RRT_H
#define RRT_H

#include <vector>
#include "config.h"

struct TreeNode {
  const TreeNode *parent;
  const Config config;
};

using tree_t = std::vector<TreeNode *>;

using obstacle_t = std::vector<point2d_t>;

struct Task {
  const Config &start;
  const Config &end;
  const std::vector<obstacle_t> &obstacles;
};

#endif
