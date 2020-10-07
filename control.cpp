#include <iostream>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>

#include "mppi/kinematic_mppi.h"
#include "mppi/stick_mppi.h"
#include "mppi/MPPILocalPlannerConfig.h"

#include "config.h"
#include "rrt.h"
#include "collision.h"
#include "graphics.h"
#include "control.h"

GraphNode *next_stepwise_target;

bool getNextConfig(Config *current, const GraphNode *path, const Task &task,
    const graph_t &graph, KinematicMPPI& mppi, const ArrayXXb& costmap, bool adaptive_carrot, bool deterministic) {

  Config target = next_stepwise_target->config;
  double d = distanceFrom(*current, target);
  if (d < 3*MAX_DIFF) {
    if (next_stepwise_target->parent == nullptr) return true;
    if (deterministic || !adaptive_carrot) next_stepwise_target = next_stepwise_target->parent;
  }

  Config command;
  if (deterministic) {
    double alpha = MAX_DIFF / d;
    Config line = diff(target, (*current));
    command = line * alpha;
  }
  else
  {
    StateArrayf x = *current;

    if (!adaptive_carrot) {
      mppi.setGoal(target);
    }

    mppi.optimize(x, costmap, graph, adaptive_carrot, task);
    command = mppi.pop(x);
  }

  assert(distanceFrom(command, Config::Zero()) < MAX_DIFF*1.001);
  *current += command;
  return false;
}

void moveProjectiles(Task &task) {
  for (projectile_t &p : task.projectiles) {
    p[0] += (randf()-0.5) * 0.02;
    p[1] += (randf()-0.5) * 0.02;
  }
}

void doControl(const GraphNode *path, Task &task, const ArrayXXb& costmap, graph_t &graph, graph_t &min_graph, bool adaptive_carrot, bool deterministic) {
  if (DYNAMIC_OBSTACLES) {
    task.projectiles.push_back({0.0, 0.0});
    task.projectiles.push_back({0.8, 0.0});
    task.projectiles.push_back({-0.8, 0.0});
    task.projectiles.push_back({0.0, 0.8});
    task.projectiles.push_back({0.0, -0.8});
  }

  Config current = path->config;
  if (!deterministic) {
    next_stepwise_target = graph.nodeForConfig(task.end);
  } else {
    next_stepwise_target = path->parent;
  }
  bool done = false;
  struct timeval tp0, tp1, tp_start;
  double secsPerFrame = 1/CONTROL_HZ;

  MPPILocalPlannerConfig mppi_config;
  mppi_config.theta_weight = THETA_WEIGHT;
  mppi_config.mppi_seed = rand();
  StickMPPI mppi(mppi_config, task);
  Array3f goal;
  goal << 0., 0., 0.;
  mppi.reset();
  mppi.setGoal(goal);

  maps_t renders({});
  sf::Texture task_space_render = render(min_graph, task);
  if (RENDER_CONFIG_SPACE) {
    for (int i = 0; i < COST_DIM_TH; i++) {
      renders.push_back(render(costmap, i));
    }
  }

  double path_cost = 0.0;
  int n_steps = 0;
  int n_steps_lost = 0;
  int n_steps_on_min_graph = 0;
  int collisions = 0;
  long total_elapsed_usecs = 0;
  bool printedWarning = false;
  gettimeofday(&tp_start, NULL);
  while (!done) {
    gettimeofday(&tp0, NULL);
    moveProjectiles(task);
    StateArrayf x = current;
    if (RENDER_CONFIG_SPACE) {
      int theta_idx = floor(current(2) / COST_RESOLUTION_TH + 0.5);
      theta_idx = theta_idx % COST_DIM_TH;
      theta_idx = (theta_idx + COST_DIM_TH) % COST_DIM_TH; // Handle negative numbers argh
      drawTexture(renders[theta_idx]);
    } else {
      drawTexture(task_space_render);
    }
    drawProjectiles(task);
    SampledTrajs asdf = mppi.recent_samples_;
    int n_trajs = asdf.X_trajs.cols();
    int traj_len = asdf.X_trajs.rows();
    for (int i = 0; i < n_trajs; i++) {
      if (i % 20 == 0) {
        StateArrayf x = asdf.X_trajs.block(traj_len-STATE_DIM, i, STATE_DIM, 1);
        drawConfig(x, sf::Color(255, 0, 0, 8), RENDER_CONFIG_SPACE);
      }
    }
    if (!deterministic) {
      drawConfig(mppi.goal_, sf::Color(0, 0, 255, 255), RENDER_CONFIG_SPACE);
      drawConfig(mppi.nearest_, sf::Color(255, 0, 0, 255), RENDER_CONFIG_SPACE);
    }
    StateArrayXf seq = mppi.rolloutNominalSeq(x);
    for (int i = seq.cols()-1; i >= 0; i--) {
      if (i % 1 == 0) {
        StateArrayf x = seq.col(i);
        if (i == 0) {
          drawConfig(x, sf::Color(0, 255, 0, 255), RENDER_CONFIG_SPACE);
        } else {
          drawConfig(x, sf::Color(128, 200, 128, 64), RENDER_CONFIG_SPACE);
        }
      }
    }

    Config prev = current;
    done = getNextConfig(&current, path, task, graph, mppi, costmap, adaptive_carrot, deterministic);
    path_cost += distanceFrom(prev, current);
    n_steps += 1;
    if (!deterministic) {
      if (nominal_terminal_node == nullptr) {
        n_steps_lost += 1;
      } else {
        drawConfig(nominal_terminal_node->config, sf::Color(255, 0, 0, 255), RENDER_CONFIG_SPACE);
        if (min_graph.nodeForConfig(nominal_terminal_node->config) != nullptr) {
          n_steps_on_min_graph += 1;
        }
      }
    }
    doneDrawingStuff();

    Config noise = randConfig() * MOTION_NOISE;
    current = current + noise;
    if (collides(current, task, BALL_RADIUS/2.0)) {
      std::cout << "YOU DIED!!!!!!\n";
      collisions += 1;
    }

    gettimeofday(&tp1, NULL);
    long totalElapsedUsecs = (tp1.tv_sec - tp_start.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp_start.tv_usec);
    if (totalElapsedUsecs > 120 * 1000 * 1000) break; // we've been stuck for two minutes
    long elapsedUsecs = (tp1.tv_sec - tp0.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp0.tv_usec);
    long desiredUsecs = secsPerFrame * 1000 * 1000;
    total_elapsed_usecs += elapsedUsecs;
    /*
    if (desiredUsecs > elapsedUsecs) {
      usleep(desiredUsecs - elapsedUsecs);
    } else if (!printedWarning) {
      printedWarning = true;
      printf("Warning: can't keep up with control freq (%ld, %ld)\n", desiredUsecs/1000, elapsedUsecs/1000);
    }
    */
  }
  printf("Result:\n"
         "  Success:               %d\n"
         "  Path length:           %f (%d collisions)\n"
         "  Control steps:         %d (%d lost)\n"
         "  Min graph steps:       %d\n"
         "  Time per control (us): %ld\n",
      done, path_cost, collisions,
      n_steps, n_steps_lost, n_steps_on_min_graph, total_elapsed_usecs / n_steps);
  std::cout << std::flush;
}
