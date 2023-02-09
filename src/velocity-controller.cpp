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

// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"




#include "../include/tb-controller/MiniPID.h"

#include "../include/tb-controller/utils.h"

using namespace std::chrono_literals;


MiniPID pid(1,0,0);

class VelocityController : public rclcpp::Node
{
  public:
  double global_radius = 5.0;
  double local_radius = 0.2;
  double origin_x = 0.0;
  double origin_y = 0.0;

    VelocityController()
    : Node("velocity_controller")
    { //! Need to fix subscriber type...its a bit more complex since they are different messages each time. See TF listener online
      subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "tf2", 10, std::bind(&VelocityController::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&VelocityController::timer_callback, this));
    }

  private:
    double output=0.1;
    void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {

      double x = msg->transform.translation.x;
      double y = msg->transform.translation.y;
      double z = msg->transform.translation.z;

      // double th = msg->transform.rotation.z;


      std::cout << "x: "<< x << " y: " << y << "z: " << z << std::endl;
      // *Find next waypoint and move to that waypoint
      
      // 1 = major circle
      // 2 = minor circle
      std::vector<utils::Pose2> intersections = utils::circle_intersection(0.0,0.0, global_radius, x, y, local_radius);
      utils::Pose2 next_waypoint = utils::find_correct_intersection(intersections[0], intersections[1], x, y);
      utils::Pose2 current_pos;
      current_pos.x = x;
      current_pos.y = y;
      // PID
      double input = utils::calculateDistance(next_waypoint,current_pos);
      double target = 0; // we want the target distance from current pos to next waypoint to be zero, i.e, we are a tthe next waypoint
      output=pid.getOutput(input,target);

    }

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
    // Publisher callback
    void timer_callback()
    {
    geometry_msgs::msg::Twist vel;

    vel.linear.x = 2;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = output;
  
    // velStamped.header.stamp = rclcpp::Node::now();

    // velStamped.twist = vel;
    RCLCPP_INFO(this->get_logger(), "Publishing vel");
    publisher_->publish(vel);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  
    size_t count_;
};



int main(int argc, char ** argv)
{

  printf("tb-controller package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityController>());
  rclcpp::shutdown();
  return 0;
}
