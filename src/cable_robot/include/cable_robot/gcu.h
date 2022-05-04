#ifndef CABLE_ROBOT_GCU_H
#define CABLE_ROBOT_GCU_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

class GCU: public rclcpp::Node
{
public:
    GCU();

private:
    void debugCallback();
    void updateCallback();
    void controlsUpdateCallback(geometry_msgs::msg::Twist::ConstSharedPtr vel);

    double t;
    double tPrev;
    double areaSideX;
    double areaSideY;
    double areaSideZ;
    double maxSpeed;
    double xSpeed;
    double ySpeed;
    double zSpeed;

    double debugInterval;
    double controlUpdateInterval;
    rclcpp::TimerBase::SharedPtr debugTimer;
    rclcpp::TimerBase::SharedPtr controlUpdateTimer;

    geometry_msgs::msg::Point realPos;
    geometry_msgs::msg::Point desPos;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controlSub;      // get contol from /cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr realPositionSub; // real position from Gazebo/IMU
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr desiredPositionPub;

    void realPositionCallback(geometry_msgs::msg::Point::ConstSharedPtr msg)
    {
        realPos = *msg;
    }
};

#endif //CABLE_ROBOT_GCU_H
