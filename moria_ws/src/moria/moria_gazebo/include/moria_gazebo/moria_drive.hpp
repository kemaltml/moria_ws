#ifndef MORIA_GAZEBO__MORIA_DRIVE_HPP_
#define MORIA_GAZEBO__MORIA_DRIVE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT 1
#define RIGHT 2

#define LINEAR_VEL 0.3
#define ANGULAR_VEL 1.5

#define GET_MORIA_DIRECTION 0
#define MORIA_DRIVE_FORWARD 1
#define MORIA_RIGHT_TURN 2
#define MORIA_LEFT_TURN 3

class MoriaDrive : public rclcpp::Node {
public:
    MoriaDrive();
    ~MoriaDrive();

private:
    //ROS topic publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    //ROS topic subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables
    double robot_pose_;
    double prev_robot_pose_;
    double scan_data_[3];

  // ROS timer
    rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
    void update_callback();
    void update_cmd_vel(double linear, double angular);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif