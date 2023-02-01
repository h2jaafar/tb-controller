#include <cstdio>
#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "std_msgs/msg/string.hpp"


#include "../include/tb-controller/MiniPID.h"
#include "../include/tb-controller/utils.h"

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


MiniPID pid_x(1,0,0);
MiniPID pid_y(1,0,0);

class MinimalSubscriber : public rclcpp::Node
{
  public:
  double global_radius = 5.0;
  double local_radius = 0.2;
  double origin_x = 0.0;
  double origin_y = 0.0;
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

      double x = msg->transform.translation.x;
      double y = msg->transform.translation.y;
      double z = msg->transform.translation.z;

      double th = msg->transform.rotation.z;


      std::cout << "x: "<< x << " y: " << y << "z: " << z << std::endl;
      // Find next waypoint and move to that waypoint
      
      // 1 = major circle
      // 2 = minor circle
      std::vector<utils::Pose2> intersections = utils::circle_intersection(0.0,0.0, global_radius, x, y, local_radius);
      utils::Pose2 next_waypoint = utils::find_correct_intersection(intersections[0], intersections[1], x, y);

      // PID
      double target_x = next_waypoint.x;
      double target_y = next_waypoint.y;

      double output_th=pid.getOutput(th,target_th);
      std::cout << "th: " << th << " output_th: " << output_th << " target_th: " << target_th << std::endl;
    }
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
};



int main(int argc, char ** argv)
{

  printf("tb-controller package\n");
  std::vector<utils::Pose2> X_ref_traj;
  X_ref_traj = utils::X_ref(5.0, 0.0, 0.0);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPublisher>());
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); // ? Not sure if this works
  rclcpp::shutdown();
  return 0;
}
