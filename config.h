#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <array>
#include <SFML/Graphics.hpp>

extern const double MIN_X;
extern const double MAX_X;
extern const double MIN_Y;
extern const double MAX_Y;
extern const double LENGTH;
extern const double BALL_RADIUS;
extern const double THETA_WEIGHT;

constexpr size_t N_BALLS = 11;

using point2d_t = std::array<double,2>;
using balls_t = std::array<point2d_t,N_BALLS>;

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
  double distanceFrom (const Config &other) const;

  balls_t getBalls() const; // for collision detection

  friend std::ostream &operator<< (std::ostream &out, const Config &c);
  friend sf::Vector2f toVector(const Config &c); // for graphics
};

#endif
