#include <custom_interfaces/srv/get_direction.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>

using namespace std::chrono_literals;

class DirectionClientTest : public rclcpp::Node {
public:
  using Direction = custom_interfaces::srv::GetDirection;

  DirectionClientTest() : Node("test_direction_node") {

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
        timer_period, std::bind(&DirectionClientTest::timer_callback, this));

    // Subscribe to laserscan to store data to be used in communicating with
    // service.
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&DirectionClientTest::laser_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Direction Service Tester Client is Ready.");
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
        request, std::bind(&DirectionClientTest::response_callback, this,
                           std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Direction Service Request Sent.");
    // // Send the request asynchronously
    // auto result_future = client_->async_send_request(request);

    // // Wait for the result
    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
    //                                        result_future) ==
    //     rclcpp::FutureReturnCode::SUCCESS) {
    //   auto response = result_future.get();
    //   // Log the service response
    //   RCLCPP_INFO(this->get_logger(), "Turn direction recommended: %s",
    //               response->direction.c_str());

    // } else {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    // }
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
  }

private:
  rclcpp::Client<Direction>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  sensor_msgs::msg::LaserScan::SharedPtr laserscan_data_;
};

int main(int argc, char **argv) {
  // Initialize the ROS communication
  rclcpp::init(argc, argv);

  // Declare the node constructor
  auto client = std::make_shared<DirectionClientTest>();
  rclcpp::spin(client);
  //   // Run the send_request() method
  //   client->send_request();

  //   // Shutdown the ROS communication
  //   rclcpp::shutdown();
  return 0;
}