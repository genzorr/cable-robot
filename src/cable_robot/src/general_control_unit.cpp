#include <iostream>
#include <chrono>
#include "cable_robot/general_control_unit.h"

GCU::GCU()
: Node("gcu"), x_(0.), y_(0.), z_(0.), xVel_(0.), yVel_(0.), zVel_(0.), controlTimerInterval_(200)
{
    RCLCPP_INFO(this->get_logger(), "Initializing General Control Unit");
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    controlTimer_ = this->create_wall_timer(std::chrono::milliseconds(controlTimerInterval_), [this] { updateCallback(); });
    controlSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos,
        std::bind(&GCU::controlCallback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "General Control Unit initialized");
}

void GCU::controlCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
    xVel_ = vel->linear.x;
    yVel_ = vel->linear.y;
    zVel_ = vel->linear.z;
}

void GCU::updateCallback()
{
    auto dt = (double)controlTimerInterval_/1000;
    x_ += xVel_ * dt;
    y_ += yVel_ * dt;
    z_ += zVel_ * dt;

    RCLCPP_INFO(this->get_logger(), "x: %.4f, y: %.4f, z: %.4f", x_, y_, z_);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto gcu = std::make_shared<GCU>();
    rclcpp::spin(gcu);
    rclcpp::shutdown();
    return 0;
}
