#include <iostream>
#include <math.h>
#include <cstdlib>
#include "config.h"

double randf() {
  return double(std::rand())/RAND_MAX;
}

Config::Config(double x, double y, double theta) : x(x), y(y), theta(theta) {
}

Config Config::randConfig() {
  return Config {MIN_X + randf()*(MAX_X-MIN_X),
                 MIN_Y + randf()*(MAX_Y-MIN_Y),
                 2*M_PI*randf()};
}

Config Config::operator+ (const Config &other) const {
  return Config {this->x + other.x, this->y + other.y, this->theta + other.theta};
}

Config Config::operator- (const Config &other) const {
  double thetaDiff = fmod(this->theta - other.theta, 2*M_PI);
  if (thetaDiff < -M_PI)
    thetaDiff += 2*M_PI;
  if (thetaDiff > M_PI)
    thetaDiff -= 2*M_PI;
  return Config {this->x - other.x, this->y - other.y, thetaDiff};
}

Config Config::operator* (double alpha) const {
  return Config {this->x * alpha, this->y * alpha, this->theta * alpha};
}

double Config::operator* (const Config &other) const {
  return this->x*other.x + this->y*other.y + THETA_WEIGHT*this->theta*other.theta;
}

double Config::distanceFrom (const Config &other) const {
  Config d = (*this) - other;
  return pow(d*d, 0.5);
}

std::ostream &operator<< (std::ostream &out, const Config &c) {
  out << "(" << c.x << ", " << c.y << ", " << c.theta << ")\n";
  return out;
}

balls_t Config::getBalls() const {
  balls_t r;
  double cos_theta = cos(this->theta);
  double sin_theta = sin(this->theta);
  r[0] = {this->x - LENGTH * 0.5 * cos_theta,
          this->y - LENGTH * 0.5 * sin_theta};
  for (size_t i = 1; i < N_BALLS; i++) {
    r[i][0] = r[i-1][0] + cos_theta*BALL_RADIUS;
    r[i][1] = r[i-1][1] + sin_theta*BALL_RADIUS;
  }
  return r;
}
