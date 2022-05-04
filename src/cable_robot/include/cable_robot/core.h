#ifndef CABLE_ROBOT_CDPR_H
#define CABLE_ROBOT_CDPR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <gazebo_msgs/msg/link_state.hpp>
#include <visp/vpHomogeneousMatrix.h>
#include "cable_robot/tda.h"

class Core: public rclcpp::Node
{
public:
    Core();

    inline void getPose(vpHomogeneousMatrix &M_) {M_ = M;}
    inline void setDesiredPose(double x, double y, double z, double tx, double ty, double tz)
    {Md = vpHomogeneousMatrix(x,y,z,tx,ty,tz);}
    inline vpPoseVector getPoseError() { return vpPoseVector(M.inverse()*Md);}

    void computeW(vpMatrix &W);
    void computeDesiredW(vpMatrix &Wd);
    void computeLength(vpColVector &L);
    void computeDesiredLength(vpColVector &Ld);

    void sendTensions(vpColVector &f);
    void sendRealPosition();

private:
    double fMin, fMax;
    vpMatrix inertia;
    int nCables;
    double mass;

    // math
    vpHomogeneousMatrix Mupd, M, Md;
    std::vector<vpTranslationVector> framePoints, platformPoints;

    double controlUpdateInterval;
    rclcpp::TimerBase::SharedPtr controlUpdateTimer;

    // Subs & pubs
    sensor_msgs::msg::JointState tensionsMsg;
    rclcpp::Subscription<gazebo_msgs::msg::LinkState>::SharedPtr platformStateSub;  // platform state from Gazebo/IMU
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tensionsPub;         // tensions to Gazebo plugin/winches
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr realPositionPub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desiredPositionSub;

    // TDA solver
    std::unique_ptr<TDA> tdaSolver;

    void updateCallback();

    void desiredPositionCallback(geometry_msgs::msg::Point::ConstSharedPtr msg)
    {
        this->setDesiredPose(msg->x, msg->y, msg->z, 0., 0., 0.);
    }

    // callback for platform state
    void platformStateCallback(gazebo_msgs::msg::LinkState::ConstSharedPtr msg)
    {
        M.insert(vpTranslationVector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
        M.insert(vpQuaternionVector(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                                     msg->pose.orientation.w));
    }
};


#endif //CABLE_ROBOT_CDPR_H
