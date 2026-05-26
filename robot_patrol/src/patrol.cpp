#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Inside the patrol.cpp file, you need to subscribe to the laser topic and
// capture the rays. For the sake of simplicity, use only the laser rays
// corresponding to the front 180° of the robot. For LaserScan there are 200
// measurements from pi to -pi. Min range is 0.01, max is 20 Assuming x axis is
// at the front, index 0 of our measurment corresponds to the rear of the robot
// (must confirm) index 50 is the right, 100 is front, 150 is left for
// laserscan_callback identify the ray with the greatest distance (which is not
// an inf) Get its corresponding angle with respect to the robot's front X axis
//     Note: The angle must be between −𝜋/2 and 𝜋/2 (in radians).
// Store that angle in a class variable named direction_.
// This angle represents the direction of the most open area, and therefore the
// direction the robot should rotate toward. Move the robot to the safest area
//     To move the robot toward the safe direction, you need to publish the
//     appropriate values to the robot’s velocity topic. Inside the patrol.cpp,
//     create a publisher to the /cmd_vel topic. Create a control callback loop
//     that runs at 10 Hz. At every iteration of the control loop, publish the
//     proper velocity command to the topic, based on the values detected by the
//     laser.
//         The linear velocity in X must always be 0.1 m/s.
//         Take the value of the variable direction_ and use it to compute the
//         proper angular velocity in Z:
//             angular velocity in z = direction_ / 2.

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol") {

    // Creat qos
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    // Subscribe to scan
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    // Publisher for movement commands
    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for controlling movement
    auto timer_period = std::chrono::milliseconds(100);
    timer_ = this->create_wall_timer(timer_period,
                                     std::bind(&Patrol::timer_callback, this));

    turn_ = false;

    RCLCPP_INFO(this->get_logger(), "Patrol node is ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_angle = msg->angle_min;
    float angle_increment = msg->angle_increment;

    // initialize index and max_distance for our callback
    size_t max_distance_index{0};
    float max_distance{0.0f};
    // Go from index 50 to 150 to find the maximum distance and index where this
    // occurs

    for (size_t i = 0 + 3; i < 112 - 3; ++i) {

      float sum = 0.0f;
      int count = 0;

      // 7-ray window: i-3 ... i+3
      for (size_t j = i - 3; j <= i + 3; ++j) {

        const float r = msg->ranges[j];

        if (std::isfinite(r)) {
          sum += r;
          count++;
        }
      }

      // skip invalid windows
      if (count == 0) {
        continue;
      }

      const float average = sum / static_cast<float>(count);

      if (average > max_distance) {
        max_distance = average;
        max_distance_index = i;
      }
    }

    for (size_t i = 336 + 3; i < 447 - 3; ++i) {

      float sum = 0.0f;
      int count = 0;

      // 7-ray window: i-3 ... i+3
      for (size_t j = i - 3; j <= i + 3; ++j) {

        const float r = msg->ranges[j];

        if (std::isfinite(r)) {
          sum += r;
          count++;
        }
      }

      // skip invalid windows
      if (count == 0) {
        continue;
      }

      const float average = sum / static_cast<float>(count);

      if (average > max_distance) {
        max_distance = average;
        max_distance_index = i;
      }
    }

    // RCLCPP_INFO(this->get_logger(),
    //             "Max distance is %.2f, max distance index is %zu",
    //             max_distance, max_distance_index);
    // Determine angle we should face w.r.t. the x axis
    direction_ = min_angle + max_distance_index * angle_increment;
    if (direction_ > M_PI) {
      direction_ -= 2 * M_PI;
    }
    // RCLCPP_INFO(this->get_logger(), "Recommended direction of travel is %.2f
    // ",
    //             direction_);

    turn_ = false;

    // Front sector near angle 0
    for (size_t i = 0; i < 10; ++i) {

      const float range = msg->ranges[i];

      if (std::isfinite(range) && range < 0.35f) {
        turn_ = true;
        break;
      }
    }

    // Front sector near 2*pi
    if (!turn_) {

      for (size_t i = 438; i < 448; ++i) {

        const float range = msg->ranges[i];

        if (std::isfinite(range) && range < 0.35f) {
          turn_ = true;
          break;
        }
      }
    }
  }

private:
  void timer_callback() {
    auto action = geometry_msgs::msg::Twist();
    action.linear.x = 0.1;
    if (turn_) {
      action.angular.z = direction_ / 2.0;
    } else {
      action.angular.z = 0.0;
    }
    publisher_vel_->publish(action);
    // RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f,
    // angular.z=%.2f",
    //             action.linear.x, action.angular.z);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
  rclcpp::TimerBase::SharedPtr timer_;
  float direction_{0.0f};
  bool turn_;
  static constexpr size_t min_laser_range_{50};
  static constexpr size_t max_laser_range_{150};
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<Patrol>();
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);
  // shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}
