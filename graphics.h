#ifndef RRT_GRAPHICS_H
#define RRT_GRAPHICS_H

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include <SFML/Graphics.hpp>
#include <Eigen/Core>

void drawGraph(const graph_t &graph, const Task &task);
void drawStuff(const Config &config, const Task &task, const graph_t &graph);
void drawConfig(const Config &config, sf::Color color = sf::Color::Green);
void doneDrawingStuff();
void animatePath(const GraphNode *path, const Task &task, const graph_t &graph,
    double movementPerFrame=0.01, double secsPerFrame=0.01);
sf::Texture render(const graph_t &graph, const Task &task, const ArrayXXb& costmap);
void drawTexture(const sf::Texture& tex);

#endif
