#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <set>
#include <ctime>
#include <stdexcept>
#include "../include/path_planner/dstar.hpp"

/**
 * Run Instructions (in src/path_planner/src)
 * 
 * $ g++ dstar.cpp dstar_visualizer.cpp
 * $ ./a.out
 * 
 */

using namespace std;

#define RESET   "\033[0m"
#define GREEN   "\033[32m"
#define RED     "\033[31m"
#define BLUE    "\033[34m"
#define GRAY    "\033[90m"
#define CYAN    "\033[36m"

#define BG_SCAN "\033[100m"

void clear_screen() {
  cout << "\033[H\033[J";
}

int main() {
  srand(time(0));

  int width = 41;
  int height = 16;

  DStar dstar_planner_(width, height);
  int start_x = 3, start_y = 3;
  int goal_x = 36, goal_y = 12;

  dstar_planner_.update_start(start_x, start_y);
  dstar_planner_.update_goal(goal_x, goal_y);

  dstar_planner_.compute_shortest_path();

  set<pair<int, int>> all_obstacles;
  set<pair<int, int>> known_obstacles;

  int num_obstacles = 350;
  while (all_obstacles.size() < num_obstacles) {
    int obstacle_x = rand() % width;
    int obstacle_y = rand() % height;

    if ((obstacle_x != start_x || obstacle_y != start_y) &&
        (obstacle_x != goal_x || obstacle_y != goal_y)) {
      all_obstacles.insert({obstacle_x, obstacle_y});
    }
  }

  set<pair<int, int>> visited;

  int curr_x = start_x;
  int curr_y = start_y;
  int prev_x = -1;
  int prev_y = -1;

  while (curr_x != goal_x || curr_y != goal_y) {
    // Finding new obstacles
    bool found_obstacle = false;
    for (int dx = -3; dx <= 3; dx++) {
      for (int dy = -3; dy <= 3; dy++) {
        int new_x = curr_x + dx;
        int new_y = curr_y + dy;

        if (new_x < 0 || width <= new_x ||
            new_y < 0 || height <= new_y) continue;

        if (all_obstacles.count({new_x, new_y}) &&
            dstar_planner_.add_obstacle(new_x, new_y)) {
          known_obstacles.insert({new_x, new_y});
          found_obstacle = true;
        }
      }
    }

    if (found_obstacle) {
      dstar_planner_.compute_shortest_path();
    }

    // Drawing map
    
    clear_screen();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        if (abs(x - curr_x) <= 3 && abs(y - curr_y) <= 3) {
          cout << BG_SCAN;
        }
        if (x == start_x && y == start_y) {
          cout << GREEN << "SSS";
        } else if (x == goal_x && y == goal_y) {
          cout << RED << "GGG";
        } else if (x == curr_x && y == curr_y) {
          cout << CYAN << "RRR";
        } else if (known_obstacles.count({x,  y})) {
          cout << GRAY << "###";
        } else {
          if (visited.count({x, y})) {
            cout << BLUE;
          }

          int num = round(dstar_planner_.get_g(x, y));
          string temp = to_string(num);
          if (num < 0) {
            temp = "INF";
          } else {
            while (temp.size() < 3) {
              temp += ".";
            }
          }
          cout << temp;
        }
        cout << RESET;
      }
      cout << endl;
    }

    // string tp;
    // cin >> tp;
    usleep(100000);

    // Computing next step
    visited.insert({curr_x, curr_y});

    try {
      dstar_planner_.update_start(curr_x, curr_y);
      dstar_planner_.compute_shortest_path();

      auto [next_x, next_y] = dstar_planner_.next_location();

      curr_x = next_x;
      curr_y = next_y;
    } catch (runtime_error& e) {
      cout << e.what() << endl;
      break;
    }
  }
}