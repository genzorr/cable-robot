#ifndef CABLE_ROBOT_CDPR_H
#define CABLE_ROBOT_CDPR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <gazebo_msgs/msg/link_state.hpp>
#include <visp/vpHomogeneousMatrix.h>
#include "cable_robot/tda.h"

class Core: public rclcpp::Node
{
public:
    Core();

    inline void getPose(vpHomogeneousMatrix &T_) {T_ = T;}
    inline void setDesiredPose(double tx, double ty, double tz, double tux, double tuy, double tuz)
    { Td = vpHomogeneousMatrix(tx, ty, tz, tux, tuy, tuz);}
    inline vpPoseVector getPoseError() {
//        vpTranslationVector t, td;
//        T.extract(t); Td.extract(td);
//        return td - t;
        return vpPoseVector(T.inverse() * Td);
    }

    void computeW(vpMatrix &W);
    void computeDesiredW(vpMatrix &Wd);
    void computeLength(vpColVector &L);
    void computeDesiredLength(vpColVector &Ld);

    void sendTensions(vpColVector &f);
    void sendRealPosition();

private:
    int nCables;
    double mass;
    double fMin, fMax;
    vpColVector a, v, vd;
    // math
    vpMatrix Ir;
    vpHomogeneousMatrix T, Td; // current and desired transformation matrices for platform state
    std::vector<vpTranslationVector> framePoints, platformPoints;

    double controlUpdateInterval;
    rclcpp::TimerBase::SharedPtr controlUpdateTimer;

    // Subs & pubs
    sensor_msgs::msg::JointState tensionsMsg;
    rclcpp::Subscription<gazebo_msgs::msg::LinkState>::SharedPtr platformStateSub;  // platform state from Gazebo/IMU
    rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr platformAccelSub;  // platform accel from Gazebo/IMU
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tensionsPub;         // tensions to Gazebo plugin/winches
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr realPositionPub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desiredPositionSub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desiredVelocitySub;

    // TDA solver
    std::unique_ptr<TDA> tdaSolver;

    void updateCallback();

    void desiredPositionCallback(geometry_msgs::msg::Point::ConstSharedPtr msg)
    {
        this->setDesiredPose(msg->x, msg->y, msg->z, 0., 0., 0.);
    }

    void desiredVelocityCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
    {
        vd[0] = msg->linear.x;
        vd[1] = msg->linear.y;
        vd[2] = msg->linear.z;
//        vd[3] = msg->angular.x;
//        vd[4] = msg->angular.y;
//        vd[5] = msg->angular.z;
        vd[3] = 0;
        vd[4] = 0;
        vd[5] = 0;
//        RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f", vd[0], vd[1], vd[2], vd[3], vd[4], vd[5]);
    }

    // callback for platform state
    void platformStateCallback(gazebo_msgs::msg::LinkState::ConstSharedPtr msg)
    {
        // Set current homogenous matrix
        T.insert(vpTranslationVector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
        T.insert(vpQuaternionVector(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                                    msg->pose.orientation.w));

        v[0] = msg->twist.linear.x;
        v[1] = msg->twist.linear.y;
        v[2] = msg->twist.linear.z;
        v[3] = msg->twist.angular.x;
        v[4] = msg->twist.angular.y;
        v[5] = msg->twist.angular.z;

        geometry_msgs::msg::Point position;
        position.x = msg->pose.position.x;
        position.y = msg->pose.position.y;
        position.z = msg->pose.position.z;
        realPositionPub->publish(position);
//        RCLCPP_INFO(this->get_logger(), "%f %f %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void platformAccelCallback(geometry_msgs::msg::Accel::ConstSharedPtr msg)
    {
        a[0] = msg->linear.x;
        a[1] = msg->linear.y;
        a[2] = msg->linear.z;
        a[3] = msg->angular.x;
        a[4] = msg->angular.y;
        a[5] = msg->angular.z;
    }
};


#endif //CABLE_ROBOT_CDPR_H
