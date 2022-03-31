#ifndef CABLE_ROBOT_GENERAL_CONTROL_UNIT_H
#define CABLE_ROBOT_GENERAL_CONTROL_UNIT_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class GCU: public rclcpp::Node
{
public:
    GCU();
private:
    void debugCallback();
    void updateCallback();
    void controlsUpdateCallback(geometry_msgs::msg::Twist::ConstSharedPtr vel);

    double x_;
    double y_;
    double z_;
    double areaSideX_;
    double areaSideY_;
    double areaSideZ_;
    double platformSide_;
    double maxSpeed_;
    double payloadMass_;
    double xSpeed_;
    double ySpeed_;
    double zSpeed_;
    double controlUpdateInterval_;
    double debugInterval_;
    rclcpp::TimerBase::SharedPtr controlUpdateTimer_;
    rclcpp::TimerBase::SharedPtr debugTimer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controlSub_;
};

#endif //CABLE_ROBOT_GENERAL_CONTROL_UNIT_H
