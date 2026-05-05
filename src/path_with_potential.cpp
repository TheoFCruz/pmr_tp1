#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <map>
#include <string>
#include <vector>

class PathWithPotential : public rclcpp::Node
{
public:
  PathWithPotential()
  : Node("path_with_potential")
  {
    // node parameters
    robot_id = this->declare_parameter<int>("robot_id", 1);
    num_robots = this->declare_parameter<int>("num_robots", 1);

    if (robot_id < 1)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid robot_id %d. Using robot_id=1.", robot_id);
      robot_id = 1;
    }

    if (num_robots < 1)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid num_robots %d. Using num_robots=1.", num_robots);
      num_robots = 1;
    }

    // publishers and subscribers
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::SensorDataQoS(),
      std::bind(&PathWithPotential::laserCallback, this, std::placeholders::_1)
    );

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10,
      std::bind(&PathWithPotential::odomCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10
    );

    createOtherRobotSubscriptions();

    // timer for the control loop
    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(LOOP_DT_MS),
      std::bind(&PathWithPotential::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Path with potential node started.");
    RCLCPP_INFO(
      this->get_logger(),
      "robot_id=%d num_robots=%d namespace=%s",
      robot_id,
      num_robots,
      this->get_namespace()
    );
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

      if (!std::isfinite(r)) r = 10;

      Eigen::Vector2d new_point(0, 0);
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
  }

  void otherOdomCallback(
    int other_robot_id,
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    other_robot_positions[other_robot_id] = Eigen::Vector2d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y
    );
  }

  void controlLoop()
  {
    // TODO: implement control logic
  }

  // ------------------ Utility Functions ---------------------

  void createOtherRobotSubscriptions()
  {
    for (int id = 1; id <= num_robots; ++id)
    {
      if (id == robot_id) continue;

      std::string topic = "/robot_" + std::to_string(id) + "/odom";

      other_odom_subs.push_back(
        this->create_subscription<nav_msgs::msg::Odometry>(
          topic,
          10,
          [this, id](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            otherOdomCallback(id, msg);
          }
        )
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Subscribing to robot_%d odom on %s",
        id,
        topic.c_str()
      );
    }
  }

  void sendVelocity(Eigen::Vector2d vel)
  {
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

  // --------------------- Variables --------------------------

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
                                                                  other_odom_subs;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr                                 control_timer;

  // laser points vector
  std::vector<Eigen::Vector2d> laser_points;
  Eigen::Vector2d              closest_point;

  // robot pose
  Eigen::Vector2d robot_pos;
  double          robot_yaw;

  // multi-robot
  int robot_id;
  int num_robots;
  std::map<int, Eigen::Vector2d> other_robot_positions;

  // consts
  const double D = 0.05;
  const int    LOOP_DT_MS = 100;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathWithPotential>());
  rclcpp::shutdown();
  return 0;
}
