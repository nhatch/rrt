#ifndef RRT_H
#define RRT_H

#include <vector>
#include "config.h"

struct TreeNode {
  const TreeNode *parent;
  Config config;
};

using tree_t = std::vector<TreeNode *>;

#endif
