#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <array>
#include <SFML/Graphics.hpp>
#include <Eigen/Core>

constexpr bool POINT_ROBOT = false;
constexpr double MIN_X = -1.0;
constexpr double MAX_X = 1.0;
constexpr double MIN_Y = -1.0;
constexpr double MAX_Y = 1.0;
constexpr double LENGTH = POINT_ROBOT ? 0.0 : 0.2;
constexpr size_t N_BALLS = POINT_ROBOT ? 1 : 11;
constexpr double BALL_RADIUS = POINT_ROBOT ? 0.02 : LENGTH / (N_BALLS - 1);
constexpr double PROJECTILE_RADIUS = 0.05;

// For the default obstacle setup, large THETA_WEIGHT (e.g. 3)
// tends to solve the problem faster.
constexpr double THETA_WEIGHT = POINT_ROBOT ? 0.0 : 0.4;

using point2d_t = std::array<double,2>;
using balls_t = std::array<point2d_t,N_BALLS>;
using ArrayXXb = Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

using Config = Eigen::Array<float, 1, 3>;

double randf();
Config randConfig();
balls_t getBalls(const Config &c);
double distanceFrom(const Config &c1, const Config &c2);
Config diff(const Config &c1, const Config &c2);

#endif
