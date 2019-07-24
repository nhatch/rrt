#include <iostream>
#include <math.h>
#include <cstdlib>
#include "config.h"

// Should be at least LENGTH^2 (see collision.h) to guarantee
// that collisions will be detected. (I think.)
// For the default obstacle setup, large THETA_WEIGHT (e.g. 3)
// tends to solve the problem faster.
const double THETA_WEIGHT = 2.;

extern const double MIN_X = -1.0;
extern const double MAX_X = 1.0;
extern const double MIN_Y = -1.0;
extern const double MAX_Y = 1.0;

void printConfig(const Config &c) {
  std::cout << "(" << c.x << ", " << c.y << ", " << c.theta << ")\n";
}

double thetaDiff(const Config &c0, const Config &c1) {
  double diff = fmod(c0.theta - c1.theta, 2*M_PI);
  if (diff < -M_PI)
    diff += 2*M_PI;
  if (diff > M_PI)
    diff -= 2*M_PI;
  return diff;
}

Config configDiff(const Config &c0, const Config &c1) {
  return Config {c0.x - c1.x, c0.y - c1.y, thetaDiff(c0, c1)};
}

double innerProduct(const Config &c0, const Config &c1) {
  return c0.x*c1.x + c0.y*c1.y + THETA_WEIGHT*c0.theta*c1.theta;
}

double configDist(const Config &c0, const Config &c1) {
  Config d = configDiff(c0, c1);
  return pow(innerProduct(d, d), 0.5);
}

double randf() {
  return double(std::rand())/RAND_MAX;
}

Config randConfig() {
  return {MIN_X + randf()*(MAX_X-MIN_X), MIN_Y + randf()*(MAX_Y-MIN_Y), 2*M_PI*randf()};
}

Config linComb(const Config &c0, const Config &line, double alpha) {
    Config newConf {alpha * line.x + c0.x,
                    alpha * line.y + c0.y,
                    alpha * line.theta + c0.theta};
    return newConf;
}

