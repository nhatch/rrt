
#include <math.h>
#include "config.h"
#include "rrt.h"
#include "collision.h"

/* We check for collisions along a trajectory by covering the stick
 * with a bunch of balls and checking each ball for collisions at
 * small intervals along the trajectory. */

bool leftOf(const point2d_t query, const point2d_t p0, const point2d_t p1, double clearance) {
  // TODO Precalculate offset and normal
  point2d_t normal {p0[1] - p1[1], p1[0] - p0[0]};
  double norm = pow(pow(normal[0], 2) + pow(normal[1], 2), 0.5);
  normal[0] /= norm;
  normal[1] /= norm;
  double offset = normal[0]*p0[0] + normal[1]*p0[1];
  double iprod = normal[0]*query[0] + normal[1]*query[1];
  return (iprod - offset > clearance);
}

bool collides(const Config &config, const Task &task, double clearance) {
  balls_t balls = getBalls(config);
  for (const obstacle_t &obs : task.obstacles) {
    for (point2d_t &ball : balls) {
      bool possibleCollision = true;
      for (size_t i = 0; possibleCollision && i < obs.size(); i++) {
        point2d_t p0 = obs[i];
        point2d_t p1 = (i == obs.size()-1) ? obs[0] : obs[i+1];
        if (leftOf(ball, p0, p1, clearance))
          possibleCollision = false;
      }
      if (possibleCollision)
        return true;
    }
  }
  for (projectile_t &p : task.projectiles) {
    for (point2d_t &ball : balls) {
      double dx = p.location(0) - ball[0];
      double dy = p.location(1) - ball[1];
      double dist = sqrt(dx*dx + dy*dy);
      if (dist < clearance + PROJECTILE_RADIUS) return true;
    }
  }
  return false;
}

Config maxConfig(const Config &c0, const Config &c1, const Task &task, double clearance, bool *noCollision) {
  double d = distanceFrom(c0, c1);
  Config line = diff(c1, c0);
  double plane_dist = pow(line(0)*line(0) + line(1)*line(1), 0.5);
  double angle_dist = LENGTH * 0.5 * abs(line(2));
  double max_safe_fraction = clearance / (plane_dist + angle_dist);
  // If we travel `max_safe_fraction` of the distance along `line`,
  // then no part of the stick will move more than `clearance` distance.
  // Thus, if two consecutive configs both have clearance `clearance`,
  // then it is guaranteed that we will not collide with an obstacle
  // while traveling between them.
  int n_frames = 1.0/max_safe_fraction + 1; // Take the ceiling to be safe
  Config current = c0;
  for (int i=1; i < n_frames+1; i++) {
    Config next = c0 + line * (double(i)/n_frames);
    if (collides(next, task, clearance)) {
      *noCollision = false;
      return current;
    }
    current = next;
  }
  *noCollision = true;
  return current; // Which in this case should equal c1
}
