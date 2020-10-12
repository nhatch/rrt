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
        if (min_dist < ETA) return existingNode; // too close to existing graph, not useful
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

GraphNode *search(const Config &start, graph_t &graph, const Task &task, double tol=ETA*2) {
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

const obstacle_t &make_blob(double x, double y, double s) {
  const obstacle_t *blob = new obstacle_t {
    {x+s, y+s}, {x+s, y-s}, {x-s, y-s}, {x-s, y+s}
  };
  return *blob;
}

void run_seed(int seed, int control_seed, std::string &mode, std::string& task_name) {
  set_seed(seed);


  // Give the vertices in clockwise order
  std::vector<obstacle_t> obstacles({});
  Config start {-0.8, -0.8, M_PI/2}, end {0.8, -0.8, M_PI/2};
  if (task_name == "gate")
  {
    const obstacle_t *obs1 = new obstacle_t {{-0.1, -1}, {-0.1, -0.05}, {0.1, -0.05}, {0.1, -1}};
    const obstacle_t *obs2 = new obstacle_t {{0.1, 1}, {0.1, 0.05}, {-0.1, 0.05}, {-0.1, 1}};
    obstacles.push_back(*obs1);
    obstacles.push_back(*obs2);
  }
  else if (task_name == "bugtrap")
  {
    const obstacle_t *obs1 = new obstacle_t {{-0.1, -1}, {-0.1, -0.05}, {0.1, -0.05}, {0.1, -1}};
    const obstacle_t *obs2 = new obstacle_t {{0.8, -0.1}, {-0.8, -0.1}, {-0.8, 0.1}, {0.8, 0.1}};
    obstacles.push_back(*obs1);
    obstacles.push_back(*obs2);
  }
  else if (task_name == "forest")
  {
    double s = 0.05;
    obstacles.push_back(make_blob(0.0,0.0,s));
    obstacles.push_back(make_blob(0.2,0.2,s));
    obstacles.push_back(make_blob(0.3,-0.7,s));
    obstacles.push_back(make_blob(0.4,0.0,s));
    obstacles.push_back(make_blob(0.5,0.2,s));
    obstacles.push_back(make_blob(0.6,-0.3,s));
    obstacles.push_back(make_blob(-0.1,-0.4,s));
    obstacles.push_back(make_blob(-0.2,0.5,s));
    obstacles.push_back(make_blob(-0.3,0.6,s));
    obstacles.push_back(make_blob(-0.4,-0.3,s));
    obstacles.push_back(make_blob(-0.5,0.0,s));
    // Can break here for sparse forest
    obstacles.push_back(make_blob(-0.3,-0.4,s));
    obstacles.push_back(make_blob(-0.1,-0.2,s));
    obstacles.push_back(make_blob(0.0,-0.9,s));
    obstacles.push_back(make_blob(0.1,-0.4,s));
    obstacles.push_back(make_blob(0.2,0.4,s));
    obstacles.push_back(make_blob(0.3,-0.5,s));
    obstacles.push_back(make_blob(-0.4,-0.8,s));
    obstacles.push_back(make_blob(-0.5,0.2,s));
    obstacles.push_back(make_blob(-0.6,0.3,s));
    obstacles.push_back(make_blob(-0.7,-0.5,s));
  }
  else if (task_name == "blob")
  {
    obstacles.push_back(make_blob(0.0,0.0,0.5));
    end(1) = 0.8; // upper right instead of lower right
  }

  std::vector<projectile_t> projectiles;
  Task task {start, end, obstacles, projectiles};
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

  std::string costmap_fname = "costmaps/costmap_" + task_name + "_" + std::to_string(seed) + ".txt";
  if (!FULL_COSTMAP) costmap_fname = "costmaps/costmap_" + task_name + "_obstacles_only.txt";
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

  if (mode == "all" || mode == "naive") {
    std::cout << "NAIVE\n";
    for (int i = 0; i < N_REPEATS; i++) {
      set_seed(control_seed++);
      doControl(path, task, costmap, graph, min_graph, false, true);
    }
  }
  if (mode == "all" || mode == "mppi_full") {
    std::cout << "MPPI (full)\n";
    for (int i = 0; i < N_REPEATS; i++) {
      set_seed(control_seed++);
      doControl(path, task, costmap, graph, min_graph, false, false);
    }
  }
  if (mode == "all" || mode == "mppi_min") {
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
  std::string task_name = "gate";
  if (argc > 1) {
    task_name = argv[1];
  }
  if (argc > 2) {
    N_SEEDS = 1;
    std::istringstream ss {argv[2]};
    ss >> seed;
  }
  unsigned int control_seed = seed;
  std::string mode = "all";
  if (argc > 3) {
    N_REPEATS = 1;
    std::istringstream ss2 {argv[3]};
    ss2 >> control_seed;
    mode = argv[4];
  }
  if (task_name != "gate" && task_name != "bugtrap" && task_name != "forest" && task_name != "blob") {
    std::cout << "Invalid task name; aborting\n";
    return 1;
  }
  if (mode != "all" && mode != "naive" && mode != "mppi_full" && mode != "mppi_min") {
    std::cout << "Invalid mode; aborting\n";
    return 2;
  }
  std::cout << "Starting seed: " << seed << std::endl;
  for (int s = 0; s < N_SEEDS; s++) {
    std::cerr << "Starting trial " << s << std::endl;
    run_seed(seed + 50*s, control_seed, mode, task_name);
  }
  return 0;
}
