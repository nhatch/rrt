
#include "graph.h"
#include "collision.h"

void getBuckets(const Config &c, int *b_x, int *b_y, int *b_th, int *d_x, int *d_y, int *d_th) {
  constexpr double BUCKET_SIDE = 0.2 * 2;
  double bucket_x = floor((c(0) - MIN_X) / BUCKET_SIDE);
  double bucket_y = floor((c(1) - MIN_Y) / BUCKET_SIDE);
  double thetaDiff = fmod(c(2), 2*M_PI);
  if (thetaDiff < 0)
    thetaDiff += 2*M_PI;
  double bucket_th = thetaDiff * THETA_WEIGHT / BUCKET_SIDE;
  *b_x = floor(bucket_x);
  *b_y = floor(bucket_y);
  *b_th = floor(bucket_th);
  *d_x = (bucket_x - *b_x < 0.5) ? -1 : 1;
  *d_y = (bucket_y - *b_y < 0.5) ? -1 : 1;
  *d_th = (bucket_th - *b_th < 0.5) ? -1 : 1;
}

int bucketsToIndex(int b_x, int b_y, int b_th) {
  return b_x*G_Y_LEN*G_TH_LEN + b_y*G_TH_LEN + b_th;
}

graph_t::graph_t() : buckets_(), num_nodes_(0) {
}

void graph_t::insert(GraphNode *node) {
  num_nodes_ += 1;
  int b_x, b_y, b_th;
  int d_x, d_y, d_th;
  getBuckets(node->config, &b_x, &b_y, &b_th, &d_x, &d_y, &d_th);
  nodelist_t &list = buckets_[bucketsToIndex(b_x, b_y, b_th)];
  list.push_back(node);
}

int graph_t::size() const {
  return num_nodes_;
}

GraphNode* graph_t::nodeForConfig(const Config &c) {
  int b_x, b_y, b_th;
  int d_x, d_y, d_th;
  getBuckets(c, &b_x, &b_y, &b_th, &d_x, &d_y, &d_th);
  const nodelist_t &list = buckets_[bucketsToIndex(b_x, b_y, b_th)];
  return list[0];
}

const GraphNode* graph_t::nearestNode(const Config &config, const Task &task, double *cost) const {
  GraphNode *min_node = nullptr;
  double min_cost = MAX_COST;
  int b_x, b_y, b_th;
  int d_x, d_y, d_th;
  getBuckets(config, &b_x, &b_y, &b_th, &d_x, &d_y, &d_th);
  for (int ix = 0; ix <= abs(d_x); ix++) {
    int x = b_x + ix*d_x;
    if (x < 0 || x >= G_X_LEN) continue;
    for (int iy = 0; iy <= abs(d_y); iy++) {
      int y = b_y + iy*d_y;
      if (y < 0 || y >= G_Y_LEN) continue;
      for (int ith = 0; ith <= abs(d_th); ith++) {
        int th = b_th + ith*d_th;
        if (th < 0 || th >= G_TH_LEN) continue;
        const nodelist_t &list = buckets_[bucketsToIndex(x, y, th)];
        for (GraphNode *node : list) {
          double d = distanceFrom(config, node->config);
          double cost = node->cost + d;
          if (d <= std::max(ETA,0.2) && cost < min_cost) {
            bool collisionFree = true;
            // If d is small enough, then it's guaranteed that the goal is not
            // on the other side of a wall. Thus we don't need to check for collisions.
            if (d > 0.2)
              maxConfig(node->config, config, task, BALL_RADIUS, &collisionFree);
            if (collisionFree) {
              min_cost = cost;
              min_node = node;
            }
          }
        }
      }
    }
  }
  *cost = min_cost;
  return min_node;
}

