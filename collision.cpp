
#include <math.h>
#include "config.h"
#include "collision.h"

/* We check for collisions along a trajectory by covering the stick
 * with a bunch of balls and checking each ball for collisions at
 * small intervals along the trajectory. */

bool leftOf(const point2d_t query, const point2d_t p0, const point2d_t p1) {
  // TODO Precalculate offset and normal
  point2d_t normal {p0[1] - p1[1], p1[0] - p0[0]};
  double norm = pow(pow(normal[0], 2) + pow(normal[1], 2), 0.5);
  normal[0] /= norm;
  normal[1] /= norm;
  double offset = normal[0]*p0[0] + normal[1]*p0[1];
  double iprod = normal[0]*query[0] + normal[1]*query[1];
  return (iprod - offset > BALL_RADIUS);
}

bool collides(const Config &config, const task_t &task) {
  balls_t balls = config.getBalls();
  for (obstacle_t obs : task) {
    for (point2d_t ball : balls) {
      bool possibleCollision = true;
      for (size_t i = 0; possibleCollision && i < obs.size(); i++) {
        point2d_t p0 = obs[i];
        point2d_t p1 = (i == obs.size()-1) ? obs[0] : obs[i+1];
        if (leftOf(ball, p0, p1))
          possibleCollision = false;
      }
      if (possibleCollision)
        return true;
    }
  }
  return false;
}

Config maxConfig(const Config &c0, const Config &c1, const task_t &task) {
  double d = c0.distanceFrom(c1);
  Config line = c1 - c0;
  // Given a non-colliding configuration (with some margin BALL_RADIUS),
  // the maximum distance (in configuration space)
  // that we can travel while remaining sure we're still non-colliding.
  // I think this is safe as long as THETA_WEIGHT >= LENGTH^2 (see config.cpp)
  // but I haven't checked the math in detail.
  double rate = BALL_RADIUS/2;
  int n_frames = d/rate + 1; // Take the ceiling to be safe
  Config current = c0;
  for (int i=1; i < n_frames+1; i++) {
    Config next = c0 + line * (double(i)/n_frames);
    if (collides(next, task))
      return current;
    current = next;
  }
  return current; // Which in this case should equal c1
}
