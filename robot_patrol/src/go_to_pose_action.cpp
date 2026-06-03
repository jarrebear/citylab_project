#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "custom_interfaces/action/go_to_pose.hpp"

constexpr double PI = 3.14159265358979323846;

class GoToPose : public rclcpp::Node {
public:
  using NavigateToPose = custom_interfaces::action::GoToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_server", options) {
    using namespace std::placeholders;

    // Action server to accept goals provide total distance travelled and give
    // feedback of distance left
    this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    // Subscriber for odometry
    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    // Publisher for motor control

    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "GoToPose action server is ready!");
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;

  // Initialize our variables
  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Pose2D goal_pos_;
  double pos_tolerance_{0.075};
  double ori_tolerance_{10.0 * PI / 180};

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Update current robot pose from odometry
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    // Have to calculate yaw here as odometry orientation is in quaternion
    current_pos_.theta =
        get_yaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const NavigateToPose::Goal> goal) {
    RCLCPP_INFO(
        this->get_logger(),
        "Received goal request with x: %.2f, y: %.2f, theta (degree): %.2f",
        goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
    using namespace std::placeholders;
    // This needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto vel = geometry_msgs::msg::Twist();

    auto goal = goal_handle->get_goal();

    goal_pos_.x = goal->goal_pos.x;
    goal_pos_.y = goal->goal_pos.y;
    goal_pos_.theta = goal->goal_pos.theta * PI / 180.0;

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }

      feedback->current_pos.x = current_pos_.x;
      feedback->current_pos.y = current_pos_.y;
      feedback->current_pos.theta = current_pos_.theta * 180 / PI;
      goal_handle->publish_feedback(feedback);
      double pos_error = calculate_distance(current_pos_, goal_pos_);

      // Calculate the desired angle to the goal
      double desired_yaw = std::atan2(goal_pos_.y - current_pos_.y,
                                      goal_pos_.x - current_pos_.x);

      // Calculate the difference between current yaw and desired yaw
      double yaw_error = desired_yaw - current_pos_.theta;

      // Normalize the yaw error to the range [-pi, pi]
      yaw_error = std::fmod(yaw_error + PI, 2 * PI) - PI;
      // Default command
      vel.linear.x = 0.0;
      vel.angular.z = 0.0;

      // If we are close enough to goal position
      if (pos_error < pos_tolerance_) {

        double final_yaw_error = goal_pos_.theta - current_pos_.theta;
        final_yaw_error = std::fmod(final_yaw_error + PI, 2 * PI) - PI;

        // Only fix final orientation
        if (std::abs(final_yaw_error) > ori_tolerance_) {
          vel.angular.z = (final_yaw_error > 0 ? 0.5 : -0.5);
          RCLCPP_INFO(
              this->get_logger(),
              "Final alignment phase. Yaw error from goal (degrees): %.2f",
              final_yaw_error * 180 / PI);
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "Goal completed with position error: %.2f, Yaw error "
                      "from goal (degrees): %.2f",
                      pos_error, final_yaw_error * 180 / PI);
          break; // exit orientation loop
        }

      } else {

        // Normal navigation behavior
        if (std::abs(yaw_error) > ori_tolerance_) {
          vel.angular.z = (yaw_error > 0 ? 0.5 : -0.5);
          RCLCPP_INFO(this->get_logger(),
                      "Turning towards goal. Yaw error (degrees): %.2f",
                      yaw_error * 180 / PI);
        } else {
          vel.linear.x = 0.2;
          RCLCPP_INFO(this->get_logger(), "Heading towards goal.");
        }
      }
      publisher_vel_->publish(vel);
      rate.sleep();
    }

    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    publisher_vel_->publish(vel);
    result->status = true;
    goal_handle->succeed(result);
  }

  double calculate_distance(const geometry_msgs::msg::Pose2D &current_pos,
                            const geometry_msgs::msg::Pose2D &goal_pos) {
    double dx = current_pos.x - goal_pos.x;
    double dy = current_pos.y - goal_pos.y;

    return std::sqrt(dx * dx + dy * dy);
  }

  double get_yaw(double x, double y, double z, double w) {

    // Calculate the yaw (orientation around the z-axis)
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GoToPose>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}