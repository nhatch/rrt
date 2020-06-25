#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <array>
#include <SFML/Graphics.hpp>
#include <Eigen/Core>

constexpr double MIN_X = -1.0;
constexpr double MAX_X = 1.0;
constexpr double MIN_Y = -1.0;
constexpr double MAX_Y = 1.0;
constexpr double LENGTH = 0.2;
constexpr size_t N_BALLS = 11;
constexpr double BALL_RADIUS = LENGTH / (N_BALLS - 1);

// Should be at least LENGTH^2 to guarantee
// that collisions will be detected. (I think.)
// For the default obstacle setup, large THETA_WEIGHT (e.g. 3)
// tends to solve the problem faster.
constexpr double THETA_WEIGHT = 0.7;

using point2d_t = std::array<double,2>;
using balls_t = std::array<point2d_t,N_BALLS>;
using ArrayXXb = Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

using Config = Eigen::Array<float, 1, 3>;

Config randConfig();
balls_t getBalls(const Config &c);
double distanceFrom(const Config &c1, const Config &c2);
Config diff(const Config &c1, const Config &c2);

/*
class Config {
public:
  double x;
  double y;
  double theta;

  Config(double x = 0.0, double y = 0.0, double theta = 0.0);

  static Config randConfig();

  Config operator* (double alpha) const; // rescaling
  Config operator+ (const Config &other) const;
  Config operator- (const Config &other) const;
  double operator* (const Config &other) const; // inner product

  balls_t getBalls() const; // for collision detection

  friend std::ostream &operator<< (std::ostream &out, const Config &c);
  friend sf::Vector2f toVector(const Config &c); // for graphics
};
*/

#endif
