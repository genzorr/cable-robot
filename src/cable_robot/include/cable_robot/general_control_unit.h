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
    void updateCallback();
    void controlCallback(geometry_msgs::msg::Twist::ConstSharedPtr vel);

    double x_;
    double y_;
    double z_;
    double xVel_;
    double yVel_;
    double zVel_;
    int controlTimerInterval_;
    rclcpp::TimerBase::SharedPtr controlTimer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controlSub_;
};

#endif //CABLE_ROBOT_GENERAL_CONTROL_UNIT_H
