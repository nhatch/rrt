#include <iostream>
#include <SFML/Graphics.hpp>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>
#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "rrt_graphics.h"

const int WINDOW_SIDE = 500;
sf::RenderWindow window(sf::VideoMode(WINDOW_SIDE, WINDOW_SIDE), "RRT visualization");
static const double offset_x = double(WINDOW_SIDE) / 2;
static const double offset_y = double(WINDOW_SIDE) / 2;
static const double scale_x = double(WINDOW_SIDE) / (MAX_X - MIN_X);
static const double scale_y = double(WINDOW_SIDE) / -(MAX_Y - MIN_Y);

sf::Vector2f toVector(double x, double y) {
  return sf::Vector2f(x*scale_x + offset_x, y*scale_y + offset_y);
}

sf::Vertex toVertex(const TreeNode *node, bool child = false) {
  Config c = node->config;
  sf::Vector2f point = toVector(c.x, c.y);
  if (child) {
    return sf::Vertex(point, sf::Color::White);
  } else {
    return sf::Vertex(point, sf::Color::Black);
  }
}

sf::Vertex toVertex(point2d_t p, bool child = false) {
  sf::Vector2f point = toVector(p[0], p[1]);
  if (child) {
    return sf::Vertex(point, sf::Color::White);
  } else {
    return sf::Vertex(point, sf::Color::Black);
  }
}

void clear() {
  if (!window.isOpen())
    exit(0);

  sf::Event event;
  while (window.pollEvent(event))
  {
    // "close requested" event: we close the window
    if (event.type == sf::Event::Closed)
        window.close();
  }

  window.clear(sf::Color::White);
}

void drawConfig(const Config &config, sf::Color color = sf::Color::Green) {
  balls_t balls = getBalls(config);

  for (point2d_t ball : balls) {
    double pixelRadius = BALL_RADIUS * scale_x;
    sf::CircleShape circle(pixelRadius);
    circle.setFillColor(color);
    sf::Vector2f pos = toVector(ball[0], ball[1]);
    pos.x -= pixelRadius;
    pos.y -= pixelRadius;
    circle.setPosition(pos);
    window.draw(circle);
  }

  sf::Vertex line[] = {toVertex(balls[0]), toVertex(balls[N_BALLS-1], true)};
  window.draw(line, 2, sf::Lines);
}

void animatePath(const TreeNode *path, const Config &start, const Config &end, const task_t &task, const tree_t &tree) {
  while (path->parent != nullptr) {
    animate(path->config, path->parent->config, start, end, task, &tree);
    path = path->parent;
  }
}

void drawTask(const task_t &task) {
  for (obstacle_t obs : task) {
    sf::ConvexShape shape;
    shape.setPointCount(obs.size());
    shape.setFillColor(sf::Color::Black);
    for (size_t i = 0; i < obs.size(); i++) {
      point2d_t p = obs[i];
      shape.setPoint(i, toVector(p[0], p[1]));
    }
    window.draw(shape);
  }
}

void animate(const Config &c, const task_t &task) {
  animate(c, c, c, c, task, nullptr);
}

void animate(const Config &c0, const Config &c1,
    const Config &start, const Config &end, const task_t &task, const tree_t *tree,
    double moveRate, double secsPerFrame) {
  double d = configDist(c0, c1);
  int n_movement_frames = d/moveRate + 1;
  Config line = configDiff(c1, c0);

  struct timeval tp0, tp1;
  for (int i=0; i < n_movement_frames; i++) {
    gettimeofday(&tp0, NULL);
    Config current = linComb(c0, line, double(i)/n_movement_frames);
    clear();
    if (tree)
      drawTree(*tree, task, false);
    drawConfig(start, sf::Color::Blue);
    drawConfig(end, sf::Color::Blue);
    drawConfig(current);
    window.display();
    gettimeofday(&tp1, NULL);
    long elapsedUsecs = (tp1.tv_sec - tp0.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp0.tv_usec);
    long desiredUsecs = secsPerFrame * 1000 * 1000;
    if (desiredUsecs > elapsedUsecs)
      usleep(desiredUsecs - elapsedUsecs);
  }
}

void drawTree(const tree_t &tree, const task_t &task, bool render) {
  if (render)
    clear();
  drawTask(task);
  for (const TreeNode *node : tree) {
    if (!node->parent)
      continue;
    sf::Vertex line[] = {toVertex(node->parent), toVertex(node, true)};
    window.draw(line, 2, sf::Lines);
  }
  if (render)
    window.display();
}

