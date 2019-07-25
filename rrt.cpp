#include <iostream>
#include <limits>
#include <ctime>
#include <math.h>

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"

void traceParents(const TreeNode *leaf) {
  while (leaf->parent != nullptr) {
    std::cout << leaf->config;
    leaf = leaf->parent;
  }
  std::cout << leaf->config;
}

/* Consider the node only, rather than the line from that node to its parents.
 * This does not use split nodes. */
double minDistance(const TreeNode *node, const Config &config) {
  return config.distanceFrom(node->config);
}

/* Uses split nodes. */
/* Assumes `node` has a parent (i.e. is not root) */
double minDistance(const TreeNode *node, const Config &config,
    bool *needsSplitNode, Config *splitConfig) {
  const Config &c0 = node->parent->config;
  const Config &c1 = node->config;
  Config line = c1 - c0;
  /* TODO: The number used for config.theta here might be off by a multiple of 2*pi
   * from the number that would give the optimal distance.
   * I don't really understand how to solve the minimum distance problem on a torus. */
  Config targetDiff = config - c0;
  double alpha = line * targetDiff / (line * line); // TODO precedence?
  if (alpha <= 0. || alpha >= 1.) {
    *needsSplitNode = false;
    return config.distanceFrom(c1);
  } else {
    *needsSplitNode = true;
    *splitConfig = c0 + line * alpha;
    return config.distanceFrom(*splitConfig);
  }
}

TreeNode *insert(tree_t &tree, const Config &config, const task_t &task) {
  TreeNode *existingNode = tree[0]; // root node
  double min_dist = config.distanceFrom(existingNode->config);
  bool needsSplitNode = false;
  bool bestNeedsSplitNode = false;
  Config splitConfig {0., 0., 0.};
  Config bestSplitConfig {0., 0., 0.};

  for (TreeNode *node : tree) {
    if (!node->parent)
      continue;
    double d = minDistance(node, config, &needsSplitNode, &splitConfig);
    if (d < min_dist) {
      min_dist = d;
      existingNode = node;
      bestSplitConfig = splitConfig;
      bestNeedsSplitNode = needsSplitNode;
    }
  }

  if (bestNeedsSplitNode) {
    TreeNode *splitNode = new TreeNode {existingNode->parent, bestSplitConfig};
    tree.push_back(splitNode);
    existingNode->parent = splitNode;
    existingNode = splitNode;
  }
  Config safeConfig = maxConfig(existingNode->config, config, task);
  TreeNode *leafNode = new TreeNode {existingNode, safeConfig};
  tree.push_back(leafNode);
  return leafNode;
}

TreeNode *search(const Config &start, tree_t &tree, const task_t &task, double tol=0.001) {
  TreeNode *current = tree[0];
  int iter = 0;
  while (start.distanceFrom(current->config) > tol) {
    Config c;
    if (++iter % 100 == 0) {
      c = start;
    } else {
      c = Config::randConfig();
    }
    current = insert(tree, c, task);
    drawTree(tree, task);
  }
  return current;
}

void destroyTree(tree_t *tree) {
  for (TreeNode *node : *tree) {
    delete node;
  }
  tree->clear();
}

int main() {
  std::cout.precision(2);
  std::cout << std::fixed;
  unsigned int seed = std::time(nullptr);
  std::cout << "Seed: " << seed << '\n';
  std::srand(seed);

  // Give the vertices in clockwise order
  obstacle_t obs1 {{-0.1, -1}, {-0.1, -0.05}, {0.1, -0.05}, {0.1, -1}};
  obstacle_t obs2 {{0.1, 1}, {0.1, 0.05}, {-0.1, 0.05}, {-0.1, 1}};
  task_t task {obs1, obs2};
  Config start {-0.8, -0.8, M_PI/2}, end {0.8, -0.8, M_PI/2};
  TreeNode *root = new TreeNode {nullptr, end};
  tree_t tree {root};
  TreeNode *path = search(start, tree, task);
  animatePath(path, start, end, task, tree);
  traceParents(path);

  destroyTree(&tree);
  return 0;
}
