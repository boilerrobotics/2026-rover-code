#include <iostream>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "path_planner/dstar.hpp"

using namespace std;

class PathPlannerNode : public rclcpp::Node {
 public:
  PathPlannerNode() : Node("path_planner_node") {
    int start_x = 1;
    int start_y = 1;
    int goal_x = 8;
    int goal_y = 8;

    DStar path_finder(10, 10);
    path_finder.update_start(start_x, start_y);
    path_finder.update_goal(goal_x, goal_y);
    path_finder.compute_shortest_path();

    path_finder.add_obstacle(4, 5);
    path_finder.compute_shortest_path();
    
    while (start_x != goal_x || start_y != goal_y) {
      cout << start_x << " " << start_y << endl;
      tie(start_x, start_y) = path_finder.next_location();
      path_finder.update_start(start_x, start_y);
      path_finder.compute_shortest_path();
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}