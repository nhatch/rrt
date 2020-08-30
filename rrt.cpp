#include <iostream>
#include <sstream>
#include <limits>
#include <ctime>
#include <math.h>

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"
#include "graph.h"
#include "control.h"
#include "arrayio.h"

const bool PLAIN_RRT = MANUAL_GRAPH || false;
double rrt_star_rad = ETA;
const int MIN_SAMPLES = 1;
int N_SEEDS = 20;
int N_REPEATS = 5;

/* Consider the node only, rather than the line from that node to its parents.
 * This does not use split nodes. */
double minDistance(const GraphNode *node, const Config &config) {
  return distanceFrom(config, node->config);
}

/* Uses split nodes. */
/* Assumes `node` has a parent (i.e. is not root) */
double minDistance(const GraphNode *node, const Config &config,
    bool *needsSplitNode, Config *splitConfig, double *baseCost) {
  const Config &c0 = node->parent->config;
  const Config &c1 = node->config;
  Config line = diff(c1, c0);
  /* TODO: The number used for config.theta here might be off by a multiple of 2*pi
   * from the number that would give the optimal distance.
   * I don't really understand how to solve the minimum distance problem on a torus. */
  Config targetDiff = diff(config, c0);

  Config c3 = line;
  Config c4 = targetDiff;
  c3(2) *= THETA_WEIGHT;
  c4(2) *= THETA_WEIGHT;
  double alpha = (c3*c4).sum() / (c3*c3).sum();

  if (alpha <= 0.) {
    *needsSplitNode = false;
    *baseCost = node->parent->cost;
    return distanceFrom(config, c0);
  } else if (alpha >= 1.) {
    *needsSplitNode = false;
    *baseCost = node->cost;
    return distanceFrom(config, c1);
  } else {
    *needsSplitNode = true;
    *splitConfig = c0 + line * alpha;
    *baseCost = distanceFrom(c0, *splitConfig) + node->parent->cost;
    return distanceFrom(config, *splitConfig);
  }
}

void value_iterate(graph_t &graph) {
  bool changed = true;
  while (changed) {
    changed = false;
    for (nodelist_t& list : graph.buckets_) {
      for (GraphNode *node : list) {
        for (size_t i = 0; i < node->children.size(); i++) {
          GraphNode *child = node->children[i];
          double path_cost = node->costs[i] + child->cost;
          if (path_cost < node->cost) {
            changed = true;
            node->parent = child;
            node->cost = path_cost;
          }
        }
      }
    }
  }
}

GraphNode *insert(graph_t &graph, const Config &config, const Task &task, GraphNode *root) {
  GraphNode *existingNode = root;
  double min_dist = distanceFrom(config, existingNode->config);
  //bool needsSplitNode = false;
  //bool bestNeedsSplitNode = false;
  //Config splitConfig {0., 0., 0.};
  //Config bestSplitConfig {0., 0., 0.};

  // Find nearest node in existing graph
  for (nodelist_t& list : graph.buckets_) {
    for (GraphNode *node : list) {
      if (!node->parent)
        continue;
      //double d = minDistance(node, config, &needsSplitNode, &splitConfig);
      double d = minDistance(node, config);
      if (d < min_dist) {
        min_dist = d;
        existingNode = node;
        //bestSplitConfig = splitConfig;
        //bestNeedsSplitNode = needsSplitNode;
      }
    }
  }
  if (MANUAL_GRAPH && config(0) < -0.7) {
    Config desiredParent {-0.3, 0.0, M_PI};
    existingNode = graph.nodeForConfig(desiredParent);
  }

  // TODO implement calculation of edge costs for split nodes?
  /*
  if (bestNeedsSplitNode) {
    GraphNode *splitNode = new GraphNode {existingNode->parent, bestSplitConfig};
    graph.push_back(splitNode);
    existingNode->parent = splitNode;
    existingNode = splitNode;
  }
  */

  // Steer toward sampled node
  double steer_frac = ETA / min_dist;
  if (steer_frac > 1.0) steer_frac = 1.0;
  Config steered = (existingNode->config) + diff(config, existingNode->config) * steer_frac;

  // Build new edges (RRT*-style)
  std::vector<GraphNode *> children({});
  std::vector<double> costs({});

  if (PLAIN_RRT) {
    bool noCollision;
    maxConfig(existingNode->config, steered, task, BALL_RADIUS, &noCollision);
    if (noCollision) {
      children.push_back(existingNode);
      // RRT search doesn't use this cost, but MPC control does later
      double distance = distanceFrom(steered, existingNode->config);
      costs.push_back(distance);
    }
  } else {
    // TODO using k-nearest RRT* might actually be less expensive
    double gamma_rrt = 4.4; // roughly, I calculate mu(xfree) < mu(x) ~= 34
    double N = graph.size() + 1;
    rrt_star_rad = gamma_rrt * pow(log(N)/N, 0.33);
    if (rrt_star_rad > ETA) rrt_star_rad = ETA;
    //std::cout << "RRT radius: " << rrt_star_rad << std::endl;
    std::vector<int> buckets_to_check({});
    graph.getBucketsAsList(steered, buckets_to_check);
    for (int idx : buckets_to_check) {
      for (GraphNode *node : graph.buckets_[idx]) {
        double distance = distanceFrom(steered, node->config);
        if (distance < rrt_star_rad) {
          bool noCollision;
          maxConfig(node->config, steered, task, BALL_RADIUS, &noCollision);
          if (noCollision) {
            children.push_back(node);
            // TODO can the cost be different from the distance?
            costs.push_back(distance);
          }
        }
      }
    }
  }

  if (children.size() > 0) {
    GraphNode *newNode = new GraphNode {steered, children, costs, nullptr, 0.};
    graph.insert(newNode);
    for (size_t i = 0; i < children.size(); i++) {
      double path_cost = costs[i] + children[i]->cost;
      if (newNode->parent == nullptr || path_cost < newNode->cost) {
        newNode->parent = children[i];
        newNode->cost = path_cost;
      }
    }
    for (size_t i = 0; i < children.size(); i++) {
      GraphNode *child = children[i];
      child->children.push_back(newNode);
      child->costs.push_back(costs[i]);
    }
    //std::cout << "New node with children: " << children.size() << std::endl;

    if (!PLAIN_RRT) value_iterate(graph);

    return newNode;
  }
  return existingNode;
}

GraphNode *search(const Config &start, graph_t &graph, const Task &task, double tol=0.001) {
  GraphNode * const root = graph.nodeForConfig(task.end);
  GraphNode *current = root;
  int iter = 0;
  std::vector<Config> manual_samples({});
  manual_samples.push_back({ 0.3, 0.0, 0.0});
  manual_samples.push_back({ 0.3, 0.0, M_PI});
  manual_samples.push_back({-0.3, 0.0, 0.0});
  manual_samples.push_back({-0.3, 0.0, M_PI});
  manual_samples.push_back(start);
  while (iter < MIN_SAMPLES || distanceFrom(start, current->config) > tol) {
    Config c;
    if (MANUAL_GRAPH) {
      c = manual_samples[iter++];
    } else {
      if (++iter % 100 == 0) {
        c = start;
      } else {
        c = randConfig();
      }
    }
    current = insert(graph, c, task, root);
    drawGraph(graph, task);
    doneDrawingStuff();
  }
  return current;
}

void destroyGraph(graph_t *graph) {
  for (nodelist_t& list : graph->buckets_) {
    for (GraphNode *node : list) {
      delete node;
    }
    list.clear();
  }
}

void set_seed(int seed) {
  std::cout << "Seed: " << seed << '\n';
  std::srand(seed);
}

void run_seed(int seed, int control_seed, int mode) {
  set_seed(seed);

  // Give the vertices in clockwise order
  const obstacle_t obs1 {{-0.1, -1}, {-0.1, -0.05}, {0.1, -0.05}, {0.1, -1}};
  const obstacle_t obs2 {{0.1, 1}, {0.1, 0.05}, {-0.1, 0.05}, {-0.1, 1}};
  const Config start {-0.8, -0.8, M_PI/2}, end {0.8, -0.8, M_PI/2};
  Task task {start, end, {obs1, obs2}};
  GraphNode *g_root = new GraphNode {end, {}, {}, nullptr, 0.0};
  graph_t graph;
  graph.insert(g_root);
  GraphNode *path = search(start, graph, task);
  //animatePath(path, task, graph);

  graph_t min_graph;
  GraphNode *curr = path;
  while (curr != nullptr) {
    min_graph.insert(curr);
    curr = curr->parent;
  }

  std::cout << "Making costmap..." << std::flush;
  ArrayXXb costmap;
  costmap.resize(COST_DIM_X, COST_DIM_Y*COST_DIM_TH);
  costmap.fill(0);

  std::string costmap_fname = "costmaps/costmap_" + std::to_string(seed) + ".txt";
  if (!FULL_COSTMAP) costmap_fname = "costmaps/costmap_obstacles_only.txt";
  if (arrayExists(costmap_fname))
  {
    loadArray(costmap, costmap_fname);
  }
  else
  {
    int counter(0), total(COST_DIM_X * COST_DIM_Y * COST_DIM_TH);
    for (unsigned int i = 0; i < COST_DIM_X; i++) {
      for (unsigned int j = 0; j < COST_DIM_Y; j++) {
        for (unsigned int k = 0; k < COST_DIM_TH; k++) {
          if (counter % 10000 == 0) {
            std::cout << counter << " of " << total << std::endl;
          }
          counter++;
          Config c;
          c <<     (i+0.5)*COST_RESOLUTION_XY + MIN_X,
                   (j+0.5)*COST_RESOLUTION_XY + MIN_Y,
                   k*COST_RESOLUTION_TH;
          if (collides(c, task, BALL_RADIUS)) {
            costmap(i, j*COST_DIM_TH + k) = 255;
          } else if (FULL_COSTMAP) {
            double min_cost;
            graph.nearestNode(c, task, &min_cost);
            costmap(i, j*COST_DIM_TH + k) = (uint8_t) (min_cost / MAX_COST * 255.0);
          }
        }
      }
    }
    saveArray(costmap, costmap_fname);
  }
  std::cout << "done.\n";

  if (mode == 0 || mode == 1) {
    std::cout << "NAIVE\n";
    for (int i = 0; i < N_REPEATS; i++) {
      set_seed(control_seed++);
      doControl(path, task, costmap, graph, min_graph, false, true);
    }
  }
  if (mode == 0 || mode == 2) {
    std::cout << "MPPI (full)\n";
    for (int i = 0; i < N_REPEATS; i++) {
      set_seed(control_seed++);
      doControl(path, task, costmap, graph, min_graph, false, false);
    }
  }
  if (mode == 0 || mode == 3) {
    std::cout << "MPPI (min)\n";
    for (int i = 0; i < N_REPEATS; i++) {
      set_seed(control_seed++);
      doControl(path, task, costmap, min_graph, min_graph, false, false);
    }
  }

  destroyGraph(&graph);
}

int main(int argc, char *argv[]) {
  //std::cout.precision(2);
  //std::cout << std::fixed;
  unsigned int seed = std::time(nullptr) % 100000;
  if (argc > 1) {
    N_SEEDS = 1;
    std::istringstream ss {argv[1]};
    ss >> seed;
  }
  unsigned int control_seed = seed;
  int mode = 0; // indicates "run all three"
  if (argc > 2) {
    N_REPEATS = 1;
    std::istringstream ss2 {argv[2]};
    ss2 >> control_seed;
    std::istringstream ss3 {argv[3]};
    ss3 >> mode;
  }
  std::cout << "Starting seed: " << seed << std::endl;
  for (int s = 0; s < N_SEEDS; s++) {
    std::cerr << "Starting seed " << s << std::endl;
    run_seed(seed + 50*s, control_seed, mode);
  }
}
