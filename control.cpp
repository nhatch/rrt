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

const GraphNode *next_stepwise_target;

Config getNextConfig(StateArrayf *current, const GraphNode *path, const Task &task,
    const graph_t &graph, KinematicMPPI& mppi, const ArrayXXb& costmap, bool adaptive_carrot, bool deterministic, bool *done) {

  Config target = next_stepwise_target->config;
  Eigen::Array<float,3,1> current_state = current->topRows(3);
  double d = distanceFrom(current_state, target);
  Config command;
  command *= 0;
  if (d < 3*MAX_DIFF) {
    if (next_stepwise_target->parent == nullptr) {
      *done = true;
      return command;
    }
    if (deterministic || !adaptive_carrot) next_stepwise_target = next_stepwise_target->parent;
  }

  if (deterministic) {
    double alpha = MAX_DIFF / d;
    Config line = diff(target, current_state);
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

  assert(distanceFrom(command, Config::Zero()) < MAX_COMMAND*1.001);
  *done = false;
  return command;
}

void moveProjectiles(Task &task) {
  for (projectile_t &p : task.projectiles) {
    p.velocity *= 0.95;
    p.velocity(0) += (randf()-0.5) * 0.01;
    p.velocity(1) += (randf()-0.5) * 0.01;
    double speed = p.velocity.matrix().norm();
    double max_speed = 0.2 * MAX_DIFF;
    if (speed > max_speed) p.velocity *= max_speed / speed;
    p.location += p.velocity;
  }
}

void doControl(const GraphNode *path, Task &task, const ArrayXXb& costmap, graph_t &graph, graph_t &min_graph, bool adaptive_carrot, bool deterministic, bool dynamic_obstacles) {
  if (dynamic_obstacles) {
    task.projectiles.clear();
    task.projectiles.push_back({0.0, 0.0});
    task.projectiles.push_back({0.8, 0.0});
    task.projectiles.push_back({-0.8, 0.0});
    task.projectiles.push_back({0.0, 0.8});
    task.projectiles.push_back({0.0, -0.8});
    task.projectiles.push_back({0.4, 0.4});
    task.projectiles.push_back({0.4, -0.4});
    task.projectiles.push_back({-0.4, -0.4});
    task.projectiles.push_back({-0.4, 0.4});
  }

  StateArrayf current;
  current *= 0;
  current.topRows(3) = task.start;

  if (!deterministic) {
    next_stepwise_target = graph.nodeForConfig(task.end);
  } else {
    next_stepwise_target = path;
  }
  bool done = false;
  struct timeval tp0, tp1;
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

  double total_cost = 0.0;
  int n_steps = 0;
  int n_steps_lost = 0;
  int n_steps_on_min_graph = 0;
  int collisions = 0;
  long total_elapsed_usecs = 0;
  bool printedWarning = false;
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
        drawConfig(x.topRows(3), sf::Color(255, 0, 0, 8), RENDER_CONFIG_SPACE);
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
          drawConfig(x.topRows(3), sf::Color(0, 255, 0, 255), RENDER_CONFIG_SPACE);
        } else {
          drawConfig(x.topRows(3), sf::Color(128, 200, 128, 64), RENDER_CONFIG_SPACE);
        }
      }
    }

    Config command = getNextConfig(&current, path, task, graph, mppi, costmap, adaptive_carrot, deterministic, &done);
    total_cost += distanceFrom(command, command*0);
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
    command += noise;
    if (SECOND_ORDER) {
      current.topRows(3) += current.bottomRows(3);
      current.bottomRows(3) += command;
      double speed = distanceFrom(current.bottomRows(3), command*0);
      if (speed > MAX_DIFF) {
        current.bottomRows(3) *= MAX_DIFF / speed;
      }
    } else {
      current.bottomRows(3) += command;
    }
    if (collides(current.topRows(3), task, BALL_RADIUS/2.0)) {
      std::cout << "YOU DIED!!!!!!\n";
      collisions += 1;
    }
    if (n_steps > 15 * 60) {
      std::cout << "TIME LIMIT REACHED\n";
      break;
    }
    if (n_steps_lost > 15 * 3) {
      std::cout << "LOST LIMIT REACHED\n";
      break;
    }

    gettimeofday(&tp1, NULL);
    long elapsedUsecs = (tp1.tv_sec - tp0.tv_sec) * 1000 * 1000 + (tp1.tv_usec - tp0.tv_usec);
    long desiredUsecs = secsPerFrame * 1000 * 1000;
    total_elapsed_usecs += elapsedUsecs;
    if (!HEADLESS) {
      if (desiredUsecs > elapsedUsecs) {
        usleep(desiredUsecs - elapsedUsecs);
      } else if (!printedWarning) {
        printedWarning = true;
        printf("Warning: can't keep up with control freq (%ld, %ld)\n", desiredUsecs/1000, elapsedUsecs/1000);
      }
    }
  }
  printf("Result:\n"
         "  Success:               %d\n"
         "  Total cost:            %f (%d collisions)\n"
         "  Control steps:         %d (%d lost)\n"
         "  Min graph steps:       %d\n"
         "  Time per control (us): %ld\n",
      done, total_cost, collisions,
      n_steps, n_steps_lost, n_steps_on_min_graph, total_elapsed_usecs / n_steps);
  std::cout << std::flush;
}
