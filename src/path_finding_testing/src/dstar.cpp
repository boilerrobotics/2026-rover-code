/**
 * James' second for trying to understand how stuff works.
 * Notes: Can pick between hashmap and 2d vector grid
 * depending on sparsity of the map; going with grid
 * Need to decide on how to implement costmap;
 * Assuming costmap[y][x] is the cost to reach a cell
 * Includes lazy deletion ()
 * 
 * Real time obstacle detection should only change whether
 * a cell is passable; lidar data is used to generate cost
 * nav_msgs/msg/OccupancyGrid only stores data about how
 * likely it is that a cell is occupied.
 */

#include <vector>
#include <numbers>
#include <cmath>
#include <queue>

using namespace std;

using Key = pair<double, double>;
using PriorityNode = pair<Key, DNode*>;

template <typename T>
using MinPriorityQueue = priority_queue<T, vector<T>, greater<T>>;

struct DNode {
  double g = INFINITY;
  double rhs = INFINITY;

  int x;
  int y;
  double z;

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

class DStar {
 private:
  // Maximum slope the robot can traverse
  static constexpr double kMaxSlope = 10;
  static constexpr double kEpsilon = 1e-6;

  MinPriorityQueue<PriorityNode> open_list_;
  
  vector<DNode> map_;
  int width_;
  int height_;

  DNode start_;
  DNode goal_;

  int km_ = 0;

 public:
  DStar(int width, int height) {
    width_ = width;
    height_ = height;

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        double z = get_elevation(x, y);
        map_.push_back(DNode(x, y));
      }
    }
  }

  double get_elevation(int x, int y) {
    // TODO: Get elevation from lidar data
  }

  /**
   * Returns cost to go from a to b
   * Considers euclidean and vertical
   * distance. Returns INFINITY if impassable
   */
  double get_cost(DNode& a, DNode& b) {
    if (b.z == INFINITY) return INFINITY:

    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double dz = abs(b.z - a.z);

    double horizontal_dist = hypot(dx, dy);

    double slope = dz / horizontal_dist;
    if (slope > kMaxSlope) return INFINITY:

    return hypot(horizontal_dist, dz);
  }
  
  /**
   * Gets 8 bordering neighbors to s;
   * Equivalent to succ() in original paper
   * Returns pointers to avoid copying
   */
  vector<Dnode*> get_neighbors(DNode& s) {
    vector<DNode*> out;

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;

        int x = s.x + dx;
        int y = s.y + dy;

        if (x < 0 || width_ <= x ||
            y < 0 || height_ <= y) continue;

        out.push_back(&map_[y * width_ + x]);
      }
    }
    
    return out;
  }

  bool is_consistent(DNode& s) {
    return abs(s.g - s.rhs) < kEpsilon;
  }

  void update_vertex(DNode& u) {
    if (u != goal_) {
      vector<DNode*> neighbors = get_neighbors(u);

      u.rhs = INFINITY;
      for (DNode* s : neighbors) {
        double cost = get_cost(s, u) + s.g;
        u.rhs = min(cost, u.rhs);
      }
    }
    
    // Lazy Deletion Part 1: Adds a new node
    if (!is_consistent(u)) {
      Key k = calculate_key(u);
      open_list_.push({u, &s});
    }
  }

  /**
   * 
   */
  void compute_shortest_path() {
    while (!open_list_.empty()) {
      const auto& [k_old, u] = open_list_.top();
      open_list_.pop();

      if (k_old >= calculate_key(start_) &&
          is_consistent(start_)) break;
      
      

    }
  }

  

  void set_start(int x, int y) {
    start_ = map_[y * width_ + x];
  }

  void set_end(int x, int y) {
    end_ = map_[y * width_ + x];
  }

  /**
   * Computes distance from a to b assuming 8
   * directional movement; Could be sped up by
   * making sqrt(2) a constant or using integers
   */
  static double heuristic(const DNode& a, const DNode& b) {
    int dx = abs(a.x - b.x);
    int dy = abs(a.y - b.y);

    int min_val = min(dx, dy);
    int max_val = max(dx, dy);

    return sqrt(2) * min_val + (max_val - min_val);
  }

  /**
   * Computes key for priority queue
   */
  Key calculate_key(const DNode& s) {
    double k2 = min(s.g, s.rhs);
    double k1 = k2 + heuristic(start_, s) + km_;
    return { k1, k2 };
  }
};

int main() {

}