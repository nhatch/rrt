#include <iostream>
#include <SFML/Graphics.hpp>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"

const int WINDOW_SIDE = 500;
sf::RenderWindow window(sf::VideoMode(WINDOW_SIDE, WINDOW_SIDE), "RRT visualization");
const double offset_x = double(WINDOW_SIDE) / 2;
const double offset_y = double(WINDOW_SIDE) / 2;
const double scale_x = double(WINDOW_SIDE) / (MAX_X - MIN_X);
const double scale_y = double(WINDOW_SIDE) / -(MAX_Y - MIN_Y);

sf::Vector2f toVector(const point2d_t &p) {
  return sf::Vector2f(p[0]*scale_x + offset_x, p[1]*scale_y + offset_y);
}

sf::Vector2f toVector(const Config &c) {
  return sf::Vector2f(c.x*scale_x + offset_x, c.y*scale_y + offset_y);
}

template <class T>
void drawLine(T p0, T p1) {
  sf::Vertex line[2];
  line[0] = sf::Vertex(toVector(p0), sf::Color::Black);
  line[1] = sf::Vertex(toVector(p1), sf::Color::White);
  window.draw(line, 2, sf::Lines);
}

void clear() {
  if (!window.isOpen())
    exit(0);

  sf::Event event;
  while (window.pollEvent(event))
  {
    if (event.type == sf::Event::Closed)
        window.close();
  }

  window.clear(sf::Color::White);
}

void drawConfig(const Config &config, sf::Color color) {
  balls_t balls = config.getBalls();

  for (point2d_t ball : balls) {
    double pixelRadius = BALL_RADIUS * scale_x;
    sf::CircleShape circle(pixelRadius);
    circle.setFillColor(color);
    sf::Vector2f pos = toVector(ball);
    pos.x -= pixelRadius;
    pos.y -= pixelRadius;
    circle.setPosition(pos);
    window.draw(circle);
  }

  drawLine(balls[0], balls[N_BALLS-1]);
}

void drawTask(const Task &task) {
  drawConfig(task.start, sf::Color::Blue);
  drawConfig(task.end, sf::Color::Blue);
  for (obstacle_t obs : task.obstacles) {
    sf::ConvexShape shape;
    shape.setPointCount(obs.size());
    shape.setFillColor(sf::Color::Black);
    for (size_t i = 0; i < obs.size(); i++) {
      shape.setPoint(i, toVector(obs[i]));
    }
    window.draw(shape);
  }
}

// We'll only draw the "best" paths in the graph, RRT* basically
void drawGraph(const graph_t &graph) {
  for (const GraphNode *node : graph) {
    if (!node->parent)
      continue;
    drawLine(node->parent->config, node->config);
  }
}

void animate(const Config &c0, const Config &c1, const Task &task, const graph_t &graph,
    double movementPerFrame, double secsPerFrame) {
  double d = c0.distanceFrom(c1);
  int n_movement_frames = d/movementPerFrame + 1;
  Config line = c1 - c0;

  struct timeval tp0, tp1;
  for (int i=0; i < n_movement_frames; i++) {
    gettimeofday(&tp0, NULL);
    Config current = c0 + line * (double(i)/n_movement_frames);
    clear();
    drawGraph(graph);
    drawTask(task);
    drawConfig(current);
    window.display();
    gettimeofday(&tp1, NULL);
    long elapsedUsecs = (tp1.tv_sec - tp0.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp0.tv_usec);
    long desiredUsecs = secsPerFrame * 1000 * 1000;
    if (desiredUsecs > elapsedUsecs)
      usleep(desiredUsecs - elapsedUsecs);
  }
}

void animatePath(const GraphNode *path, const Task &task, const graph_t &graph,
    double movementPerFrame, double secsPerFrame) {
  while (path->parent != nullptr) {
    animate(path->config, path->parent->config, task, graph, movementPerFrame, secsPerFrame);
    path = path->parent;
  }
}

void drawGraph(const graph_t &graph, const Task &task) {
  clear();
  drawGraph(graph);
  drawTask(task);
}

sf::Texture render(const graph_t &graph, const Task &task, const ArrayXXb& costmap) {
  drawGraph(graph, task);
  sf::Texture tex;
  if (!tex.create(WINDOW_SIDE, WINDOW_SIDE)) {
    std::cout << "Texture creation failed!\n";
  }
  tex.update(window);
  return tex;
}

void drawTexture(const sf::Texture& tex) {
  window.draw(sf::Sprite(tex));
}

void doneDrawingStuff() {
  window.display();
}
