#include <iostream>
#include <SFML/Graphics.hpp>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"

void drawConfig(const Config &config, sf::Color color, bool point_only) {
}

void drawGraph(const graph_t &graph) {
}

void animatePath(const GraphNode *path, const Task &task, const graph_t &graph,
    double movementPerFrame, double secsPerFrame) {
}

void drawGraph(const graph_t &graph, const Task &task) {
}

sf::Texture render(const ArrayXXb& costmap, int theta_offset) {
  sf::Texture tex;
  return tex;
}

sf::Texture render(const graph_t &graph, const Task &task) {
  sf::Texture tex;
  return tex;
}

void drawTexture(const sf::Texture& tex) {
}

void doneDrawingStuff() {
}
