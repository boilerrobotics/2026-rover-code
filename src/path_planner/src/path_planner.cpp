#include <iostream>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "path_planner/dstar.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "obstacle_interfaces/msg/obstacle_locations.hpp"

using namespace std;
using namespace geometry_msgs::msg;

class PathPlannerNode : public rclcpp::Node {
 public:
  enum class Mode {
    IDLE = 0,
    ARUCO = 1,
    GLOBAL = 2,
    OBJECT = 3
  }

  PathPlannerNode() : Node("path_planner_node"), mode_(MODE::IDLE) {
    waypoints_pub_ = this->create_publisher<nav_mags::msg::Path>("next_waypoints", 10);
    obstacles_sub_ = this->create_subscription<obstacle_interfaces::msg::ObstacleLocations>(
      "obstacle_locations", 10,
      [this](const obstacle_interfaces::msg::ObstacleLocations::SharedPtr msg) {
        this->obstacles_callback(msg);
      }
    );
    position_sub_ = this->create_subscription<Point32>(
      "current_position", 10,
      [this](const Point32::SharedPtr msg) {
        this->position_callback(msg);
      }
    );
    orientation_sub_ = this->create_susbcription<Quaternion>(
      "current_orientation", 10,
      [this](const Quaternion::SharedPtr msg) {
        this.orientation_callback(msg);
      }
    )

  }

 private:
  rclcpp::Subscription<Point32>::SharedPtr global_destination_sub_;
  DStar dstar_planner_;
  Pose pose_;

  int dstarx_;
  int dstary_;

  void set_mode(int mode) {
    if (mode == 0) {
      global_destination_sub_ = this->create_subscription<Point32>(
        "global_destination", 10,
        [this](const Point32::SharedPtr msg) {
          this->global_destination_callback(msg);
        }
      );
    }
  }

  Point32 local_to_global(double rel_x, double rel_y) {
    double dx = pose_.orientation.x;
    double dy = pose_.orientation.y;

    double curr_x = pose_.position.x;
    double curr_y = pose_.position.y;

    double distance = hypot(dx, dy);
    double cos_theta = dx / distance;
    double sin_theta = dy / distance;

    double global_x = curr_x + (rel_x * cos_theta - rel_y * sin_theta);
    double global_y = curr_y + (rel_x * sin_theta + rel_y * cos_theta);

    return Point32{global_x, global_y};
  }

  void publish_path() {

  }

  void position_callback(const Point32::SharedPtr msg) const {
    pose_.position.x = msg->x;
    pose_.position.y = msg->y;

    if (hypot(msg->x - dstarx_, ))

    if ((int) msg->x != dstarx_ || msg->y != dstary_) {
      dstar_planner_.update_start()
    }
  }

  void orientation_callback(const Quaternion::SharedPtr msg) const {
    pose_.orientation.x = msg->x;
    pose_.orientation.y = msg->y;
  }

  void obstacles_callback(const obstacle_interfaces::msg::ObstacleLocations::SharedPtr msg) const {
    for (Point32 location : msg->locations) {
      Point32 global = local_to_global(location->x, location->y);
      dstar_planner_.add_obstacle(global.x, global.y);
    }
    publish_path();
  }

  void global_destination_callback(const Point32::SharedPtr msg) const {
    Point32 global = local_to_global(msg->x, msg->y);
    dstar_planner_.update_goal(global.x, global.y);
    publish_path();
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}