/**
 * @file walking_turtle.cpp
 * @author Kautilya Reddy Chappidi
 * @brief Implements a ROS2 node for a custom walker algorithm with rosbag recording.
 * @version 0.1
 * @date 2023-12-01
 * Copyright 2023 Kautilya Reddy Chappidi
 */

#include <rosbag2_cpp/writer.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @class TurtleWalker
 * @brief Represents the state and behavior of a turtle walker robot.
 */
class TurtleWalker {
 public:
  TurtleWalker() : linear_velocity_(0.2), angular_velocity_(0.0) {}

  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges[0] < 0.6) {
      linear_velocity_ = 0.0;
      angular_velocity_ = 0.2;  // angular velocity for rotation
    } else {
      linear_velocity_ = 0.1;  // linear velocity for unique forward movement
      angular_velocity_ = 0.0;
    }
  }

  double getLinearVelocity() const { return linear_velocity_; }
  double getAngularVelocity() const { return angular_velocity_; }

 private:
  double linear_velocity_;
  double angular_velocity_;
};

/**
 * @class TWalkerNode
 * @brief ROS2 node for the turtle walker algorithm with rosbag recording.
 */
class TWalkerNode : public rclcpp::Node {
 public:
  TWalkerNode() : Node("turtle_walker_node") {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&TWalkerNode::laserCallback, this, std::placeholders::_1));
    turtle_walker_ = std::make_shared<TurtleWalker>();
    this->declare_parameter("record_bag", 1);
    bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
    bag_writer_->open("turtle_walker_bag");

    // Initialize the twist message
    twist_msg_.linear.x = turtle_walker_->getLinearVelocity();
    twist_msg_.angular.z = turtle_walker_->getAngularVelocity();
  }

 private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    turtle_walker_->update(msg);

    twist_msg_.linear.x = turtle_walker_->getLinearVelocity();
    twist_msg_.angular.z = turtle_walker_->getAngularVelocity();

    cmd_vel_pub_->publish(twist_msg_);

    if (this->get_parameter("record_bag").as_int() == 1) {
      rclcpp::Time time_stamp = this->now();
      bag_writer_->write(twist_msg_, "cmd_vel", time_stamp);
      bag_writer_->write(*msg, "scan", time_stamp);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  geometry_msgs::msg::Twist twist_msg_;
  std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;
  std::shared_ptr<TurtleWalker> turtle_walker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto turtle_node = std::make_shared<TWalkerNode>();
  rclcpp::spin(turtle_node);
  rclcpp::shutdown();
  return 0;
}
