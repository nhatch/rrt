#include <iostream>
#include <math.h>
#include <cstdlib>
#include "config.h"

double randf() {
  return double(std::rand())/RAND_MAX;
}


Config randConfig() {
  Config r;
  r <<  MIN_X + randf()*(MAX_X-MIN_X),
        MIN_Y + randf()*(MAX_Y-MIN_Y),
        2*M_PI*(randf()-0.5);
  return r;
}

Config diff (const Config &c1, const Config &c2) {
  Config d = c1 - c2;
  double thetaDiff = fmod(d(2), 2*M_PI);
  if (thetaDiff < -M_PI)
    thetaDiff += 2*M_PI;
  if (thetaDiff > M_PI)
    thetaDiff -= 2*M_PI;
  d(2) = thetaDiff;
  return d;
}

double distanceFrom (const Config &c1, const Config &c2) {
  Config d = diff(c1, c2);
  d(2) *= THETA_WEIGHT;
  return pow((d*d).sum(), 0.5);
}

balls_t getBalls(const Config &c) {
  balls_t r;
  double cos_theta = cos(c(2));
  double sin_theta = sin(c(2));
  r[0] = {c(0) - LENGTH * 0.5 * cos_theta,
          c(1) - LENGTH * 0.5 * sin_theta};
  for (size_t i = 1; i < N_BALLS; i++) {
    r[i][0] = r[i-1][0] + cos_theta*BALL_RADIUS;
    r[i][1] = r[i-1][1] + sin_theta*BALL_RADIUS;
  }
  return r;
}
