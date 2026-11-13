/**
 * James' second for trying to understand how stuff works.
 * Notes: Can pick between hashmap and 2d vector grid
 * depending on sparsity of the map; going with grid
 * Need to decide on how to implement costmap;
 * Assuming costmap[y][x] is the cost to reach a cell
 * 
 * Real time obstacle detection should only change whether
 * a cell is passable; lidar data is used to generate cost
 * nav_msgs/msg/OccupancyGrid only stores data about how
 * likely it is that a cell is occupied.
 * 
 * I chose to handle lazy deletion via version counter;
 * alternatives are hash set. Standard lazy deletion check
 * (k_old != calculate_key(s)) fails because of km_
 * adjustments. Version counter = extra 20 mb of ram, 
 * but unordered_map has overhead and could be slower.
 * Untested.
 * 
 * Optional minor optimizations: put ++ before variable
 * rather than after.
 * 
 * Order of Operations
 * 1. After D* Lite is constructed, update_start must be
 * called followed by update_goal
 * 2. compute_shortest_path should be called when changes
 * are made to start, end, or the map before next_location
 * 3. add_obstacles must be called after update_start
 * (or just call compute_shortest_path after add_obstacles)
 */

#include <vector>
#include <numbers>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <iostream>
#include <tuple>

#include "../include/path_planner/dstar.hpp"

using namespace std;

DStar::DStar(int width, int height) {
  width_ = width;
  height_ = height;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double z = get_elevation(x, y);
      map_.push_back(DNode(x, y, z));
    }
  }
}

bool DStar::is_initialized() {
  return start_ != nullptr && goal_ != nullptr;
}

int DStar::to_index(int x, int y) {
  if (x < 0 || x >= width_ || y < 0 || y >= height_) {
    throw runtime_error("Index out of bounds. Received (" + to_string(x) + "," + to_string(y) + ")");
  }
  return y * width_ + x;
}

pair<int, int> DStar::get_start() {
  if (start_ == nullptr) {
    throw runtime_error("Start not set.");
  }
  return {start_->x, start_->y};
}

double DStar::get_elevation(int x, int y) {
  // TODO: Get elevation from lidar data
  return 0.0;
}

/**
 * Returns cost to go from a to b
 * Considers euclidean and vertical
 * distance. Returns INFINITY if impassable
 */
double DStar::get_cost(DNode& a, DNode& b) {
  if (b.z == INFINITY || a.z == INFINITY) return INFINITY;

  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double dz = abs(b.z - a.z);

  double horizontal_dist = hypot(dx, dy);

  double slope = dz / horizontal_dist;
  if (slope > kMaxSlope) return INFINITY;

  return hypot(horizontal_dist, dz);
}

/**
 * Gets 8 bordering neighbors to s;
 * Equivalent to succ() in original paper
 * Returns pointers to avoid copying
 */
vector<DNode*> DStar::get_neighbors(DNode& s) {
  vector<DNode*> out;

  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;

      int x = s.x + dx;
      int y = s.y + dy;

      if (x < 0 || width_ <= x ||
          y < 0 || height_ <= y) continue;

      out.push_back(&map_[to_index(x, y)]);
    }
  }
  
  return out;
}

bool DStar::is_consistent(DNode& s) {
  return (s.g == INFINITY && s.rhs == INFINITY)
          || abs(s.g - s.rhs) < kEpsilon;
}

void DStar::update_vertex(DNode& u) {
  // Check the address of u to check if
  // it is the same as that of goal_
  // Works because both refer to the same
  // memory in map_
  if (&u != goal_) {
    u.rhs = INFINITY;
    for (DNode* s : get_neighbors(u)) {
      double cost = get_cost(*s, u) + s->g;
      u.rhs = min(cost, u.rhs);
    }
  }
  
  // Lazy Deletion Part 1: Adds a new node
  if (!is_consistent(u)) {
    Key k = calculate_key(u);
    open_list_.push({k, &u, ++u.version});
  } else {
    // u has become consistent all previous
    // entries in open_list_ must be invalidated
    u.version++;
  }
}

/**
 * Iterates through open_list_ until start location
 * is fully consistent
 * NOTE: u is a pointer here
 */
void DStar::compute_shortest_path() {
  if (!is_initialized()) {
    throw runtime_error("Not initialized before computing shortest path.");
  }
  while (!open_list_.empty()) {
    auto [k_old, u, version] = open_list_.top();
    
    // Lazy Deletion Part 2: Skips outdated nodes
    if (version != u->version) {
      open_list_.pop();
      continue;
    }

    if (k_old >= calculate_key(*start_) &&
        is_consistent(*start_)) break;
    
    open_list_.pop();

    // Outdated because of km_ update
    Key k_new = calculate_key(*u);
    if (k_old < k_new) {
      open_list_.push({k_new, u, ++(u->version)});
      continue;
    }
    
    // Locally over-consistent -> Locally consistent
    if (u->g > u->rhs) {
      u->g = u->rhs;
      u->version++;

      for (DNode* s : get_neighbors(*u)) {
        update_vertex(*s);
      }
    }
    // Locally under-consistent -> Locally over-consistent
    else if (u->g < u->rhs) {
      u->g = INFINITY;

      for (DNode* s : get_neighbors(*u)) {
        update_vertex(*s);
      }
      update_vertex(*u);
    }
  }
}

/**
 * Potentially change to take directions dx
 * and dy (to reflect rover movement)
 */
void DStar::update_start(int x, int y) {
  DNode* new_start = &map_[to_index(x, y)];
  if (start_ != nullptr && start_ != new_start) {
    km_ += heuristic(*start_, *new_start);
  }
  start_ = new_start;
}

void DStar::update_goal(int x, int y) {
  if (start_ == nullptr) {
    throw runtime_error("Start not set before updating goal.");
  }

  DNode* new_goal = &map_[to_index(x, y)];
  if (new_goal == goal_) return;
  goal_ = new_goal;

  open_list_ = MinPriorityQueue<PriorityNode>();
  km_ = 0.0;

  for (int i = 0; i < map_.size(); i++) {
    map_[i].rhs = INFINITY;
    map_[i].g = INFINITY;
    map_[i].version = 0;
  }

  goal_->rhs = 0.0;
  
  Key k = calculate_key(*goal_);
  open_list_.push({k, goal_, ++(goal_->version)});
}

/**
 * Returns true if the new obstacle is added;
 * otherwise obstacle already exists-->no replan
 * needed.
 */
bool DStar::add_obstacle(int x, int y) {
  DNode* u = &map_[to_index(x, y)];

  if (u->z == INFINITY) return false;

  u->z = INFINITY;
  u->g = INFINITY;
  u->rhs = INFINITY;
  u->version++;

  update_vertex(*u);

  for (DNode* s : get_neighbors(*u)) {
    update_vertex(*s);
  }

  return true;
}

/**
 * Gets the next n waypoints for where the rover
 * should move; Assumes that all nodes are consistent
 * and that the start node is set; Stops early if it
 * reaches the waypoint.
 */
vector<pair<int, int>> DStar::next_n_locations(int n) {

  if (start_ == goal_) {
    return {{start_->x, start_->y}};
  }

  DNode* curr = start_;
  vector<pair<int, int>> waypoints;
  for (int i = 0; i < n; i++) {

    double min_val = INFINITY;
    DNode* next = nullptr;

    for (DNode* s : get_neighbors(*curr)) {
      double cost = get_cost(*curr, *s) + s->g;
      if (cost < min_val) {
        min_val = cost;
        next = s;
      }
    }

    if (min_val == INFINITY) {
      throw runtime_error("No valid next location found.");
    }

    curr = next;
    waypoints.push_back({curr->x, curr->y});

    if (curr == goal_) {
      break;
    }
  }
  return waypoints;
}

/**
 * Gets the coordinates the next location the
 * rover should move; Assumes that all nodes
 * are consistent; Maybe will change to a Point32
 * data type; Assuming start node is set
 */
pair<int, int> DStar::next_location() {

  if (start_ == goal_) {
    return {start_->x, start_->y};
  }

  double min_val = INFINITY;
  pair<int, int> next = {start_->x, start_->y};

  for (DNode* s : get_neighbors(*start_)) {
    double cost = get_cost(*start_, *s) + s->g;
    if (cost < min_val) {
      min_val = cost;
      next = {s->x, s->y};
    }
  }

  if (min_val == INFINITY) {
    throw runtime_error("No valid next location found.");
  }

  return next;
}

/**
 * Computes distance from a to b assuming 8
 * directional movement; Could be sped up by
 * making sqrt(2) a constant or using integers
 */
double DStar::heuristic(const DNode& a, const DNode& b) {
  int dx = abs(a.x - b.x);
  int dy = abs(a.y - b.y);

  int min_val = min(dx, dy);
  int max_val = max(dx, dy);

  return sqrt(2) * min_val + (max_val - min_val);
}

/**
 * Computes key for priority queue
 */
Key DStar::calculate_key(const DNode& s) {
  double k2 = min(s.g, s.rhs);
  double k1 = k2 + heuristic(*start_, s) + km_;
  return { k1, k2 };
}