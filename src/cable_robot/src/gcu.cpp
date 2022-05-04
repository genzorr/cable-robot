#include <chrono>
#include <functional>
#include "cable_robot/gcu.h"

GCU::GCU()
: Node("gcu"), t(0.), tPrev(0.),
  areaSideX(0.), areaSideY(0.), areaSideZ(0.),
  maxSpeed(0.), xSpeed(0.), ySpeed(0.), zSpeed(0.),
  debugInterval(0.), controlUpdateInterval(0.)
{
    RCLCPP_INFO(this->get_logger(), "Initializing General Control Unit");
    // Load params
    areaSideX = this->declare_parameter<double>("areaSideX", 0.);
    areaSideY = this->declare_parameter<double>("areaSideY", 0.);
    areaSideZ = this->declare_parameter<double>("areaSideZ", 0.);
    maxSpeed = this->declare_parameter<double>("maxSpeed", 0.);
    debugInterval = this->declare_parameter<double>("debugInterval", 0.);
    controlUpdateInterval = this->declare_parameter<double>("controlUpdateInterval", 0.);

    realPos.x = 0; realPos.y = 0; realPos.z = 2;
    desPos.x = 0; desPos.y = 0; desPos.z = 2;

    debugTimer = this->create_wall_timer(
            std::chrono::milliseconds(int(debugInterval * 1000)), [this]
            { debugCallback(); });
    controlUpdateTimer = this->create_wall_timer(
            std::chrono::microseconds(int(controlUpdateInterval * 1000000)), [this]
            { updateCallback(); });

    // Subs & pubs
    controlSub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 5, std::bind(&GCU::controlsUpdateCallback, this, std::placeholders::_1));
    realPositionSub = this->create_subscription<geometry_msgs::msg::Point>(
            "real_position", 5, std::bind(&GCU::realPositionCallback, this, std::placeholders::_1));
    desiredPositionPub = this->create_publisher<geometry_msgs::msg::Point>("desired_position", 5);

    RCLCPP_INFO(this->get_logger(), "General Control Unit initialized");
}

void GCU::debugCallback()
{
    RCLCPP_DEBUG(this->get_logger(),
                 "des: (%.3f, %.3f, %.3f), real: (%.3f, %.3f, %.3f)",
                 desPos.x, desPos.y, desPos.z, realPos.x, realPos.y, realPos.z);
//    RCLCPP_DEBUG(this->get_logger(), "dt: %.15f, t: %.15f, tPrev: %.15f", t - tPrev, t, tPrev);
}

void GCU::controlsUpdateCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
    // Update and restrain speed
    if (vel->linear.x <= maxSpeed)
        xSpeed = vel->linear.x;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for x speed");
    if (vel->linear.y <= maxSpeed)
        ySpeed = vel->linear.y;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for y speed");
    if (vel->linear.z <= maxSpeed)
        zSpeed = vel->linear.z;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for z speed");
}

void updateCoordinate(double &x, double &dx, double min, double max)
{
    if (x + dx < min)
        x = min;
    else if (x + dx > max)
        x = max;
    else
        x += dx;
}

void GCU::updateCallback()
{
//    tPrev = t;
//    t = (double)this->now().nanoseconds() / 1e+9;
//    t = this->get_clock()->now().seconds();

//    // Update and restrain coordinate
    auto dt = controlUpdateInterval;
    double dx = xSpeed * dt, dy = ySpeed * dt, dz = zSpeed * dt;
    updateCoordinate(desPos.x, dx, -areaSideX / 2, areaSideX / 2);
    updateCoordinate(desPos.y, dy, -areaSideY / 2, areaSideY / 2);
    updateCoordinate(desPos.z, dz, 0, areaSideZ);

    desiredPositionPub->publish(desPos);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto gcu = std::make_shared<GCU>();

    rclcpp::spin(gcu);
    rclcpp::shutdown();
    return 0;
}
