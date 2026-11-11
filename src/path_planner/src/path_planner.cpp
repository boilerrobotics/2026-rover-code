#include <iostream>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "path_planner/dstar.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
  };

  PathPlannerNode() : Node("path_planner_node"), dstar_planner_(1600, 1600) {
    lookahead_distance_ = 10;

    this->declare_parameter("mode", 0);
    mode_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    mode_handle_ = mode_sub_->add_parameter_callback("mode",
      [this](const rclcpp::Parameter& msg) {
        this->mode_callback(msg);
      }
    );

    waypoints_pub_ = this->create_publisher<nav_msgs::msg::Path>("next_waypoints", 10);
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
    orientation_sub_ = this->create_subscription<Quaternion>(
      "current_orientation", 10,
      [this](const Quaternion::SharedPtr msg) {
        this->orientation_callback(msg);
      }
    );
  }

 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<obstacle_interfaces::msg::ObstacleLocations>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<Point32>::SharedPtr position_sub_;
  rclcpp::Subscription<Quaternion>::SharedPtr orientation_sub_;

  std::shared_ptr<rclcpp::ParameterEventHandler> mode_sub_;
  shared_ptr<rclcpp::ParameterCallbackHandle> mode_handle_;

  rclcpp::Subscription<Point32>::SharedPtr global_destination_sub_;
  
  DStar dstar_planner_;
  Pose pose_;

  Mode mode_;
  int lookahead_distance_;

  void mode_callback(const rclcpp::Parameter& p) {
    mode_ = (Mode) p.as_int();
    if (mode_ == Mode::GLOBAL) {
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

    Point32 point;
    point.x = global_x;
    point.y = global_y;
    return point;
  }

  void publish_path() {
    if (!dstar_planner_.is_initialized()) {
      return;
    }

    dstar_planner_.compute_shortest_path();
    vector<pair<int, int>> waypoints = dstar_planner_.next_n_locations(lookahead_distance_);
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    
    for (auto [x, y] : waypoints) {
      PoseStamped pose_stamped;
      pose_stamped.header = path_msg.header;
      
      pose_stamped.pose.position.x = x;
      pose_stamped.pose.position.y = y;

      pose_stamped.pose.orientation.w = 1;

      path_msg.poses.push_back(pose_stamped);
    }

    waypoints_pub_->publish(path_msg);
  }

  void position_callback(const Point32::SharedPtr msg) {
    pose_.position.x = msg->x;
    pose_.position.y = msg->y;

    if (!dstar_planner_.is_initialized()) {
      dstar_planner_.update_start((int) msg->x, (int) msg->y);
      return;
    }
    
    auto [dstar_x, dstar_y] = dstar_planner_.get_start();

    // Convert from indices to meter distances
    double prev_x = dstar_x + 0.5;
    double prev_y = dstar_y + 0.5;

    double buffer = 0.15;

    if (abs(msg->x - prev_x) > 0.5 + buffer ||
        abs(msg->y - prev_y) > 0.5 + buffer) {
      
      dstar_planner_.update_start((int) msg->x, (int) msg->y);
      publish_path();
    }
  }

  void orientation_callback(const Quaternion::SharedPtr msg) {
    pose_.orientation.x = msg->x;
    pose_.orientation.y = msg->y;
  }

  void obstacles_callback(const obstacle_interfaces::msg::ObstacleLocations::SharedPtr msg) {
    for (Point32 location : msg->locations) {
      Point32 global = local_to_global(location.x, location.y);
      dstar_planner_.add_obstacle(global.x, global.y);
    }
    publish_path();
  }

  void global_destination_callback(const Point32::SharedPtr msg) {
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