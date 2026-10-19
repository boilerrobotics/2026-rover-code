/**
 * James' second for trying to understand how stuff works.
 * Notes: Can pick between hashmap and 2d vector grid
 * depending on sparsity of the map; going with grid
 * Need to decide on how to implement costmap;
 * Assuming costmap[y][x] is the cost to reach a cell
 */

#include <vector>
#include <numbers>
#include <cmath>
#include <queue>

using namespace std;

using Key = pair<double, double>;
using PriorityNode = pair<Key, int>;

template <typename T>
using MinPriorityQueue = priority_queue<T, vector<T>, greater<T>>;

struct DNode {
  double g = INFINITY;
  double rhs = INFINITY;
  int x;
  int y;

  DNode() = default;

  DNode(int x, int y) {
    this->x = x;
    this->y = y;
  }

  bool operator == (const DNode& other) const {
    return x == other.x && y == other.y;
  }
};

class DStar {
 private:
  MinPriorityQueue<PriorityNode> pq_;
  
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
        map_.push_back(DNode(x, y));
      }
    }
  }

  bool is_collision(int x, int y) {

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

        if (isCollision(x, y)) continue;

        out.push_back(&map_[y * width_ + x]);
      }
    }
    
    return out;
  }

  void update_vertex(DNode& s) {
    if (s != goal_) {
      vector<DNode*> neighbors = get_neighbors(s);
      for (DNode* neighbor : neighbors) {

      }
    }
  }

  /**
   * 
   */
  void compute_shortest_path() {

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