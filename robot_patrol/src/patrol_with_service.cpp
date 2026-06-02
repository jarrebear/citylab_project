#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <custom_interfaces/srv/get_direction.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class PatrolServiceClient : public rclcpp::Node {
public:
  using Direction = custom_interfaces::srv::GetDirection;

  PatrolServiceClient() : Node("test_direction_node") {

    // Create the Service Client object
    std::string name_service = "/direction_service";
    client_ = this->create_client<Direction>(name_service);

    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }

    // Create a timer for periodically calling service
    auto timer_period = std::chrono::milliseconds(500);
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&PatrolServiceClient::timer_callback, this));

    // Subscribe to laserscan to store data to be used in communicating with
    // service.
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&PatrolServiceClient::laser_callback, this,
                  std::placeholders::_1));

    // Publisher for movement commands
    publisher_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Patrol Service Client is Ready.");
  }

  void send_request() {

    // Create a Direction request
    auto request = std::make_shared<Direction::Request>();

    // If no laserscan data is received don't send request
    if (!laserscan_data_) {
      RCLCPP_WARN(this->get_logger(), "No LaserScan received yet");
      return;
    }

    request->laser_data = *laserscan_data_;

    auto future = client_->async_send_request(
        request, std::bind(&PatrolServiceClient::response_callback, this,
                           std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Direction Service Request Sent.");
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laserscan_data_ = msg;
    return;
  }

  void timer_callback() { send_request(); }

  void response_callback(rclcpp::Client<Direction>::SharedFuture future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Direction Service Response Received: %s",
                response->direction.c_str());
    RCLCPP_INFO(this->get_logger(), "Turn direction recommended: %s",
                response->direction.c_str());

    turn_direction_ = response->direction.c_str();

    // Initialize our message to publish to the cmd_vel topic
    auto action = geometry_msgs::msg::Twist();

    action.linear.x = 0.1;

    if (turn_direction_ == "forward") {
      action.angular.z = 0.0;

    } else if (turn_direction_ == "right") {
      action.angular.z = -0.5;

    } else if (turn_direction_ == "left") {
      action.angular.z = 0.5;
    }

    publisher_vel_->publish(action);
  }

private:
  rclcpp::Client<Direction>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
  sensor_msgs::msg::LaserScan::SharedPtr laserscan_data_;
  std::string turn_direction_;
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<PatrolServiceClient>();
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);
  // shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}
