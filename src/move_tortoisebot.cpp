#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

class MoveTortoisebotNode : public rclcpp::Node {
private:
  // member variables
  double px;
  double py;
  double yaw;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_g1;
  rclcpp::CallbackGroup::SharedPtr callback_g2;

  // ros objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_control;

  // member method
  void subscriber_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // reading current position from /odom topic
    this->px = msg->pose.pose.position.x;
    this->py = msg->pose.pose.position.y;

    // reading current orientation from /odom topic
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // convert quaternion into euler angles
    this->yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  void timer_control_callback() {
    // warn user about execution
    int counter = 5;
    while (counter > 0) {
      // node feedback
      RCLCPP_WARN(this->get_logger(), "Starting Execution in %d", counter);
      counter--;

      // halt thread for few seconds
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // move forward
    RCLCPP_WARN(this->get_logger(), "Action : Move forward!");
    this->move_robot(0.1, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // move backward
    RCLCPP_WARN(this->get_logger(), "Action : Move backward!");
    this->move_robot(-0.1, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // move anti-clockwize
    RCLCPP_WARN(this->get_logger(), "Action : Move anti-clockwize!");
    this->move_robot(0.0, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // move clockwize
    RCLCPP_WARN(this->get_logger(), "Action : Move clockwize!");
    this->move_robot(0.0, -0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // stop movement
    RCLCPP_WARN(this->get_logger(), "Action : Halt!");
    this->move_robot(0.0, 0.0);

    // halt the execution
    this->timer_control->cancel();
    rclcpp::shutdown();
  }

  void move_robot(double x, double z) {
    // define message
    auto message = geometry_msgs::msg::Twist();

    // node feedback
    RCLCPP_INFO(this->get_logger(), "Current : [x %f, y %f, yaw %f]", this->px,
                this->py, this->yaw);

    // assign values
    message.linear.x = x;
    message.angular.z = z;

    // publish velocity
    this->publisher_cmd_vel->publish(message);
  }

public:
  // constructor
  MoveTortoisebotNode() : Node("move_tortoisebot_node") {

    // callback groups objects
    callback_g1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_g2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // ros node options
    rclcpp::SubscriptionOptions sub_callback_g1;
    sub_callback_g1.callback_group = callback_g1;

    // ros objects
    this->subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MoveTortoisebotNode::subscriber_odom_callback, this,
                  std::placeholders::_1),
        sub_callback_g1);
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->timer_control = this->create_wall_timer(
        std::chrono::seconds(1 / 1),
        std::bind(&MoveTortoisebotNode::timer_control_callback, this),
        callback_g2);
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MoveTortoisebotNode>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}