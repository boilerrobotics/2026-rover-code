#pragma once

#include <vector>
#include <queue>

using namespace std;

struct DNode {
  double g = INFINITY;
  double rhs = INFINITY;

  int x;
  int y;
  double z;

  // Must be large to avoid overflow
  uint64_t version = 0;

  DNode() = default;

  DNode(int x, int y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }

  bool operator == (const DNode& other) const {
    return x == other.x && y == other.y;
  }
};

template <typename T>
using MinPriorityQueue = priority_queue<T, vector<T>, greater<T>>;
using Key = pair<double, double>;

struct PriorityNode {
  Key key;
  DNode* node;
  uint64_t version;

  bool operator > (const PriorityNode& other) const {
    return key > other.key;
  }
};

class DStar {
 private:
  // Maximum slope the robot can traverse
  static constexpr double kMaxSlope = 10;
  static constexpr double kEpsilon = 1e-6;

  MinPriorityQueue<PriorityNode> open_list_;
  
  vector<DNode> map_;
  int width_;
  int height_;

  DNode* start_ = nullptr;
  DNode* goal_ = nullptr;

  double km_ = 0.0;
 public:
  DStar(int width, int height);
  int to_index(int x, int y);
  double get_elevation(int x, int y);
  double get_cost(DNode& a, DNode& b);
  vector<DNode*> get_neighbors(DNode& s);
  bool is_consistent(DNode& s);
  void update_vertex(DNode& u);
  void compute_shortest_path();
  void update_start(int x, int y);
  void update_goal(int x, int y);
  void add_obstacle(int x, int y);
  pair<int, int> next_location();
  static double heuristic(const DNode& a, const DNode& b);
  Key calculate_key(const DNode& s);
};