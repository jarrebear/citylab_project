#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <custom_interfaces/srv/get_direction.hpp>

constexpr double PI = 3.14159265358979323846;

class DirectionService : public rclcpp::Node {
public:
  using Direction = custom_interfaces::srv::GetDirection;

  DirectionService() : Node("direction_server_node") {

    // Create a service that will handle turn commands
    std::string name_service = "/direction_service";
    direction_service_ = this->create_service<Direction>(
        name_service, std::bind(&DirectionService::direction_callback, this,
                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Direction Service Ready");
  }

private:
  void direction_callback(const std::shared_ptr<Direction::Request> request,
                          std::shared_ptr<Direction::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Service Requested");

    // Read in values from our service request
    sensor_msgs::msg::LaserScan laser_data = request->laser_data;

    if (first_scan_) {
      min_angle_ = laser_data.angle_min;
      max_angle_ = laser_data.angle_max;
      angle_increment_ = laser_data.angle_increment;
      range_min_ = laser_data.range_min;
      range_max_ = laser_data.range_max;
      first_scan_ = false;
    }

    //
    double total_dist_sec_front =
        sumSector(laser_data, direction_angles_["sec_front"][0],
                  direction_angles_["sec_front"][1]);
    double total_dist_sec_left =
        sumSector(laser_data, direction_angles_["sec_left"][0],
                  direction_angles_["sec_left"][1]);
    double total_dist_sec_right =
        sumSector(laser_data, direction_angles_["sec_right"][0],
                  direction_angles_["sec_right"][1]);

    // RCLCPP_INFO(this->get_logger(), "Total distance front: %.2f",
    //             total_dist_sec_front);
    // RCLCPP_INFO(this->get_logger(), "Total distance right: %.2f",
    //             total_dist_sec_right);
    // RCLCPP_INFO(this->get_logger(), "Total distance left: %.2f",
    //             total_dist_sec_left);

    if ((total_dist_sec_front >= total_dist_sec_left) &&
        (total_dist_sec_front >= total_dist_sec_right)) {
      RCLCPP_INFO(this->get_logger(), "Service Response: forward");
      response->direction = "forward";
      RCLCPP_INFO(this->get_logger(), "Service Completed");
      return;

    } else if ((total_dist_sec_left >= total_dist_sec_front) &&
               (total_dist_sec_left >= total_dist_sec_right)) {
      RCLCPP_INFO(this->get_logger(), "Service Response: left");
      response->direction = "left";
      RCLCPP_INFO(this->get_logger(), "Service Completed");
      return;

    } else {
      RCLCPP_INFO(this->get_logger(), "Service Response: right");
      response->direction = "right";
      RCLCPP_INFO(this->get_logger(), "Service Completed");
      return;
    }

    // // Based on turn direction give action proper angular velocity
    // if (direction == "left") {
    //   action.angular.z = angular_velocity;
    // } else if (direction == "right") {
    //   action.angular.z = -angular_velocity;
    // } else {
    //   RCLCPP_INFO(this->get_logger(), "Invalid turn direction");
    //   response->success = false;
    //   return;
    // }

    // rclcpp::Duration duration = rclcpp::Duration::from_seconds(turn_time);
    // auto start_time = this->now();

    // rclcpp::Rate rate(10); // 10 Hz

    // // Publish for set time at 10 Hz
    // while ((this->now() - start_time) < duration) {
    //   publisher_vel_->publish(action);
    //   rate.sleep();
    // }

    // // Halt after set time and give the success response
    // publisher_vel_->publish(geometry_msgs::msg::Twist());
    // response->success = true;
  }

private:
  rclcpp::Service<Direction>::SharedPtr direction_service_;

  // Define angles (in degrees) for each sector
  std::map<std::string, std::array<double, 2>> direction_angles_{
      {"sec_front", {-30.0 * PI / 180, 30.0 * PI / 180}},
      {"sec_left", {30.0 * PI / 180, 90.0 * PI / 180}},
      {"sec_right", {-90.0 * PI / 180, -30.0 * PI / 180}}};

  float min_angle_;
  float max_angle_;
  float angle_increment_;
  float range_min_;
  float range_max_;
  bool first_scan_ = true;
  double penalty_ = -1.0;
  double penalty_range_ = 0.35;

private:
  double sumSector(const sensor_msgs::msg::LaserScan &scan, double a0,
                   double a1) {
    // Shift angle into scan's convention if needed
    if (a0 < min_angle_)
      a0 += 2.0 * PI;
    if (a1 < min_angle_)
      a1 += 2.0 * PI;

    int n = static_cast<int>(scan.ranges.size());
    int i0 = static_cast<int>(std::round((a0 - min_angle_) / angle_increment_));
    int i1 = static_cast<int>(std::round((a1 - min_angle_) / angle_increment_));
    i0 = std::max(0, std::min(i0, n - 1));
    i1 = std::max(0, std::min(i1, n - 1));

    double sum = 0.0;
    if (i0 <= i1) {
      for (int i = i0; i <= i1; ++i) {
        double r = scan.ranges[static_cast<std::size_t>(i)];
        if (std::isfinite(r) && r >= range_min_ && r <= range_max_) {
          if (r < penalty_range_) {
            sum += r + penalty_;
          } else {
            sum += r;
          }
        }
      }
    } else {
      // Wrap-around: e.g. front sector in [0, 2π] scan
      for (int i = i0; i < n - 1; ++i) {
        double r = scan.ranges[static_cast<std::size_t>(i)];
        if (std::isfinite(r) && r >= range_min_ && r <= range_max_) {
          if (r < penalty_range_) {
            sum += r + penalty_;
          } else {
            sum += r;
          }
        }
      }
      for (int i = 0; i <= i1; ++i) {
        double r = scan.ranges[static_cast<std::size_t>(i)];
        if (std::isfinite(r) && r >= range_min_ && r <= range_max_) {
          if (r < penalty_range_) {
            sum += r + penalty_;
          } else {
            sum += r;
          }
        }
      }
    }
    return sum;
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}