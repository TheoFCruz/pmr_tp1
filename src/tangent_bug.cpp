#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <vector>

class TangentBug : public rclcpp::Node
{
public:
  TangentBug()
  : Node("tangent_bug")
  {
    // publishes and subscribers
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&TangentBug::laserCallback, this, std::placeholders::_1)
    );

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&TangentBug::odomCallback, this, std::placeholders::_1)
    );

    goal_sub = this->create_subscription<geometry_msgs::msg::Point>(
      "/goal",
      10,
      std::bind(&TangentBug::goalCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    // timer for the control loop
    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TangentBug::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Tangent bug node started.");
  }

private:

  // ---------------------- Callbacks -------------------------
  
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // clear old points
    laser_points.clear();

    // used for robot -> map reference
    Eigen::Rotation2Dd r_yaw = Eigen::Rotation2Dd(robot_yaw); 

    // get closest obstacle point
    double min_dist = 1000;

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

        if (r < min_dist)
        {
          closest_point = new_point;
          min_dist = r;
        }
      }

      // next angle
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

    // check if goal was reached
    if (goal_received && (robot_pos - goal).norm() <= TOLERANCE)
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping control loop");
      sendVelocity(Eigen::Vector2d::Zero());
      goal_received = false;
    }
  }

  void goalCallback(geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: (%.2lf, %.2lf)", msg->x, msg->y);
    goal = Eigen::Vector2d(msg->x, msg->y);
    goal_received = true;
  }

  void controlLoop()
  {
    if (!goal_received) return;

    // if path to goal is clear, send direct velocity
    if (isGoalClear())
    {
      Eigen::Vector2d vel = goal - robot_pos;
      vel = vel.normalized() * SPEED;
      sendVelocity(vel);
      return;
    } 

    // get discontinuity points 
    std::vector<Eigen::Vector2d> discontinuities = getDiscontinuities();

    // calculate the heuristic to determine the best discontinuity point
    Eigen::Vector2d disc_point = calculateHeuristic(discontinuities);
    sendVelocity((disc_point - robot_pos).normalized()*SPEED);

    // compare d_reach to d_followed
    //   if d_reach <= d_followed, store it as d_followed and follow behavior 1
    //   if d_reach > d_followed, follow behavior 2
    // behavior 1 (move to goal):
    //   send velocity towards best discontinuity point
    // behavior 2 (boundary follow):
    //   send velocity towards the next discontinuity point 

    // NOTE: before sending the velocity it should be checked if the robot 
    // is to close to an obstacle and avoid a collision. I imagine removing the
    // velocity component normal to the obstacle might be enough.
  }

  // ----------------- Utility Functions ----------------------
  
  bool isGoalClear()
  {
    Eigen::Vector2d goal_dir = (goal - robot_pos).normalized();
    double goal_dist = (goal-robot_pos).norm();

    Eigen::Vector2d robot_to_obst = {0,0};
    for (const auto& point : laser_points)
    {
      robot_to_obst = point - robot_pos;
      double projection = robot_to_obst.dot(goal_dir); // obstacle dist in the dir of the goal
      
      // skip obstacles behind robot and after goal
      if (projection <= 0 || projection > goal_dist) continue; 

      double normal_dist = (robot_to_obst - projection*goal_dir).norm();
      if (normal_dist <= SAFE_RADIUS) return false;
    }
    return true;
  }

  void sendVelocity(Eigen::Vector2d vel)
  {
    vel = getSafeVelocity(vel);

    double v_x = vel.x();
    double v_y = vel.y();

    // feedback linearization
    double v = (v_x * std::cos(robot_yaw)) + (v_y * std::sin(robot_yaw));
    double w = (-v_x * std::sin(robot_yaw) + v_y * std::cos(robot_yaw)) / D;

    // ros2 msg
    geometry_msgs::msg::Twist vel_twist;
    vel_twist.linear.x = v;
    vel_twist.angular.z = w;

    cmd_vel_pub->publish(vel_twist);
  }

  Eigen::Vector2d getSafeVelocity(Eigen::Vector2d desired_vel)
  {
    // return if no obstacles
    if (laser_points.empty()) return desired_vel; 

    // return if not too close
    double range = (closest_point - robot_pos).norm();
    if (range > SAFE_RADIUS) return desired_vel;

    // return if velocity isn't going towards obstacle
    RCLCPP_INFO(this->get_logger(), "Teste1");
    if (desired_vel.dot(closest_point - robot_pos) <= 0) return desired_vel;

    // remove normal component
    RCLCPP_INFO(this->get_logger(), "Too close to the obstacle, setting safe velocity");
    Eigen::Vector2d n = (closest_point - robot_pos).normalized();
    double projection = desired_vel.dot(n);
    Eigen::Vector2d result_vel = (desired_vel - projection*n).normalized()*SPEED;

    return result_vel;
  }

  std::vector<Eigen::Vector2d> getDiscontinuities()
  {
    // returns instantly if there are 0 or 1 point to avoid size_t underflow
    if (laser_points.size() < 2) return laser_points;

    // compares the distance between one point and the next
    const double THRESHOLD = 0.3;
    std::vector<Eigen::Vector2d> discontinuities;
    for (size_t i = 0; i < laser_points.size() - 1; i++)
    {
      if ((laser_points[i] - laser_points[i+1]).norm() >= THRESHOLD)
      {
        discontinuities.push_back(laser_points[i]);
        discontinuities.push_back(laser_points[i+1]);
      }
    }

    // compare first and last points
    if ((laser_points[0] - laser_points[laser_points.size() - 1]).norm() >= THRESHOLD)
    {
      discontinuities.push_back(laser_points[0]);
      discontinuities.push_back(laser_points[laser_points.size() - 1]);
    }

    return discontinuities;
  }

  Eigen::Vector2d calculateHeuristic(std::vector<Eigen::Vector2d> &discontinuities)
  {
    if (discontinuities.empty()) return goal;

    Eigen::Vector2d best = discontinuities[0];
    double min_cost = 1000;

    for (const auto& point : discontinuities)
    {
      double cost = (goal - point).norm() + (point - robot_pos).norm();
      
      if (cost < min_cost)
      {
        best = point;
        min_cost = cost;
      }
    }

    return best;
  }

  // --------------------- Variables --------------------------
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr   goal_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr                                 control_timer;

  // laser points vector
  std::vector<Eigen::Vector2d> laser_points; 
  Eigen::Vector2d              closest_point;

  // robot and goal
  Eigen::Vector2d goal;
  Eigen::Vector2d robot_pos;
  double          robot_yaw;

  // flags
  bool goal_received = false;

  // consts
  const double SPEED = 0.5;
  const double SAFE_RADIUS = 0.5;
  const double D = 0.05;
  const double TOLERANCE = 0.05;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TangentBug>());
  rclcpp::shutdown();
  return 0;
}
