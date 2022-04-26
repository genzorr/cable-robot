#include <iostream>
#include <chrono>
#include <functional>
#include "cable_robot/general_control_unit.h"

GCU::GCU()
: Node("gcu"), x_(0.), y_(0.), z_(0.),
areaSideX_(0.), areaSideY_(0.), areaSideZ_(0.),
platformSide_(0.), maxSpeed_(0.), payloadMass_(0.),
xSpeed_(0.), ySpeed_(0.), zSpeed_(0.),
controlUpdateInterval_(0.), debugInterval_(0.)
{
    RCLCPP_INFO(this->get_logger(), "Initializing General Control Unit");

    // Load params
    {
        this->declare_parameter<double>("x", 0.0);
        this->declare_parameter<double>("y", 0.0);
        this->declare_parameter<double>("z", 0.0);
        this->declare_parameter<double>("areaSideX", 0.);
        this->declare_parameter<double>("areaSideY", 0.);
        this->declare_parameter<double>("areaSideZ", 0.);
        this->declare_parameter<double>("platformSide", 0.);
        this->declare_parameter<double>("maxSpeed", 0.);
        this->declare_parameter<double>("payloadMass", 0.);
        this->declare_parameter<double>("debugInterval", 0.);
        this->declare_parameter<double>("controlUpdateInterval", 0.);
        this->get_parameter("x", x_);
        this->get_parameter("y", y_);
        this->get_parameter("z", z_);
        this->get_parameter("areaSideX", areaSideX_);
        this->get_parameter("areaSideY", areaSideY_);
        this->get_parameter("areaSideZ", areaSideZ_);
        this->get_parameter("platformSide", platformSide_);
        this->get_parameter("maxSpeed", maxSpeed_);
        this->get_parameter("payloadMass", payloadMass_);
        this->get_parameter("debugInterval", debugInterval_);
        this->get_parameter("controlUpdateInterval", controlUpdateInterval_);
    }

    debugTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(debugInterval_*1000)), [this] { debugCallback(); });
    controlUpdateTimer_ = this->create_wall_timer(
            std::chrono::microseconds(int(controlUpdateInterval_*1000000)), [this] { updateCallback(); });

    // Subs
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    controlSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos,
        std::bind(&GCU::controlsUpdateCallback, this, std::placeholders::_1)
    );
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "General Control Unit initialized");
}

void GCU::controlsUpdateCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
    // Update and restrain speed
    if (vel->linear.x <= maxSpeed_)
        xSpeed_ = vel->linear.x;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for x speed");
    if (vel->linear.y <= maxSpeed_)
        ySpeed_ = vel->linear.y;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for y speed");
    if (vel->linear.z <= maxSpeed_)
        zSpeed_ = vel->linear.z;
    else
        RCLCPP_WARN(this->get_logger(), "Reached limit for z speed");
}

void restrainSpeed(double &x, double &dx, double &restrain)
{
    if (x + dx < 0.)
        x = 0.;
    else if (x + dx > restrain)
        x = restrain;
    else
        x += dx;
}

void GCU::debugCallback()
{
    RCLCPP_DEBUG(this->get_logger(), "x: %.4f, y: %.4f, z: %.4f", x_, y_, z_);
}

void GCU::updateCallback()
{
    // Update and restrain coordinate
    auto dt = controlUpdateInterval_;
    double dx = xSpeed_ * dt, dy = ySpeed_ * dt, dz = zSpeed_ * dt;
    restrainSpeed(x_, dx, areaSideX_);
    restrainSpeed(y_, dy, areaSideY_);
    restrainSpeed(z_, dz, areaSideZ_);

    // Send tf2 data
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();;
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = z_;
    tfBroadcaster_->sendTransform(t);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto gcu = std::make_shared<GCU>();
    rclcpp::spin(gcu);
    rclcpp::shutdown();
    return 0;
}
