#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>

class TangentBug : public rclcpp::Node
{
public:
  TangentBug()
  : Node("tangent_bug")
  {
    // publishes and subscribers
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&TangentBug::laserCallback, this, std::placeholders::_1)
    );

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&TangentBug::laserCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    // timer for the control loop
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TangentBug::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Tangent bug node started.");
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    laser_points.clear();

    Eigen::Rotation2Dd r_yaw(robot_yaw);

    double current_angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      // get points relative to the robot
      double r = msg->ranges[i];

      if (std::isfinite(r))
      {
        Eigen::Vector2d new_point(0,0);
        new_point.x() = r * std::cos(current_angle);
        new_point.y() = r * std::sin(current_angle);

        // transform the points to the map using the robot pose
        new_point = r_yaw * new_point + robot_pos;

        laser_points.push_back(new_point);

      }

      // incrementa o angulo
      current_angle += msg->angle_increment;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // get position
    robot_pos.x() = msg->pose.pose.position.x;
    robot_pos.y() = msg->pose.pose.position.y;

    // get quaternion and extract yaw
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    robot_yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  void timerCallback()
  {
    auto twist = geometry_msgs::msg::Twist();

    // TODO: implement Tangent Bug logic here using last_scan_
    // 
    // check if line to goal intercepts known obstacle
    //   if it doesn't, send velocity in the goal direction and return
    // get discontinuity points 
    // calculate the heuristic to determine the best discontinuity point
    // compare d_reach to d_followed
    //   if d_reach <= d_followed, store it as d_followed and follow behavior 1
    //   if d_reach > d_followed, follow behavior 2
    // behavior 1 (move to goal):
    //   send velocity towards best discontinuity point
    // behavior 2 (boundary follow):
    //   send velocity towards the next discontinuity point 

    // before publishing, process velocity (feedback linearization and safe distance)
    cmd_vel_pub_->publish(twist);
  }

  // variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr                                 control_timer_;

  std::vector<Eigen::Vector2d> laser_points; 

  Eigen::Vector2d goal;
  Eigen::Vector2d robot_pos;
  double          robot_yaw;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TangentBug>());
  rclcpp::shutdown();
  return 0;
}
