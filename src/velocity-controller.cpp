#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node
{
public:
  VelocityPublisher()
  : Node("velocity_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&VelocityPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::Twist vel;

    vel.linear.x = 2;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0.6;
  
    // velStamped.header.stamp = rclcpp::Node::now();

    // velStamped.twist = vel;
    RCLCPP_INFO(this->get_logger(), "Publishing vel");
    publisher_->publish(vel);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  
  size_t count_;
};



// ! TODO: Make sure this is being run ... ros spin below
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

      double x = msg->transform.translation.x;
      double y = msg->transform.translation.y;
      double z = msg->transform.translation.z;

      // double pose[3] = {x, y, z};
      std::cout << "x: "<< x << " y: " << y << "z: " << z << std::endl;
      printf("Heard a message");
    }
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{

  printf("hello world tb-controller package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPublisher>());
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); // ? Not sure if this works
  rclcpp::shutdown();
  return 0;
}
