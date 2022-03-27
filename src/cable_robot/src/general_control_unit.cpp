#include <iostream>
#include <chrono>
#include "cable_robot/general_control_unit.h"

GCU::GCU()
: Node("gcu"), x_(0.), y_(0.), z_(0.), xVel_(0.), yVel_(0.), zVel_(0.), controlTimerInterval_(200)
{
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    controlTimer_ = this->create_wall_timer(std::chrono::milliseconds(controlTimerInterval_), [this] { updateCallback(); });
    controlSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos,
        std::bind(&GCU::controlCallback, this, std::placeholders::_1)
    );
}

void GCU::controlCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
    xVel_ = vel->linear.x;
    yVel_ = vel->linear.y;
    zVel_ = vel->linear.z;
}

void GCU::updateCallback()
{
    auto dt = (double)controlTimerInterval_;
    x_ += xVel_ * dt;
    y_ += yVel_ * dt;
    z_ += zVel_ * dt;

    std::cout << x_ << " " << y_<< " " << z_ << std::endl;
}

int main(int argc, char ** argv)
{
    // Add ros logs
    std::cout << "General Control Unit" << std::endl;

    rclcpp::init(argc, argv);
    auto gcu = std::make_shared<GCU>();
    rclcpp::spin(gcu);
    rclcpp::shutdown();
    return 0;
}
