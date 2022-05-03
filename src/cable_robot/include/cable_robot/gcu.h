#ifndef CABLE_ROBOT_GCU_H
#define CABLE_ROBOT_GCU_H

#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/msg/link_state.hpp>
#include <visp/vpHomogeneousMatrix.h>
#include "cable_robot/tda.h"

class GCU: public rclcpp::Node
{
public:
    GCU();

    inline bool ok() {return cables_ok && platform_ok && trajectory_ok ;}

    inline void setDesiredPose(double x, double y, double z, double tx, double ty, double tz)
    {Md_ = vpHomogeneousMatrix(x,y,z,tx,ty,tz);}
    inline void getPose(vpHomogeneousMatrix &M) {M = M_;}
    inline void getVelocity(vpColVector &v) {v = v_;}
    inline void getDesiredPose(vpHomogeneousMatrix &M) {M = Md_;}
    inline vpPoseVector getPoseError() { return vpPoseVector(M_.inverse()*Md_);}
    inline vpPoseVector getDesiredPoseError(vpHomogeneousMatrix &M_p, vpHomogeneousMatrix &M_c)
    {return vpPoseVector(M_c.inverse()*M_p);}

    inline void getDesiredVelocity(vpColVector &v) {v = v_d;}
    inline void getDesiredAcceleration(vpColVector &a) {a = a_d;}

    void sendTensions(vpColVector &f);

    // get model parameters
    inline void tensionMinMax(double &fmin, double &fmax) {fmin = fMin; fmax = fMax;}

    // structure matrix
    //void computeW(vpMatrix &W);
    void computeW(vpMatrix &W);
    void computeDesiredW(vpMatrix &Wd);
    void computeLength(vpColVector &L);
    void computeDesiredLength(vpColVector &Ld);

private:
    void debugCallback();
    void updateCallback();
    void controlsUpdateCallback(geometry_msgs::msg::Twist::ConstSharedPtr vel);

    double t_;
    double t_prev_;
    double x_;
    double y_;
    double z_;
    double areaSideX_;
    double areaSideY_;
    double areaSideZ_;
    double platformSide_;
    double maxSpeed_;
    double mass_;
    double xSpeed_;
    double ySpeed_;
    double zSpeed_;
    double controlUpdateInterval_;
    double debugInterval_;
    rclcpp::TimerBase::SharedPtr controlUpdateTimer_;
    rclcpp::TimerBase::SharedPtr debugTimer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controlSub_;

    // subscriber to gazebo data
    bool cables_ok, platform_ok, trajectory_ok;
    sensor_msgs::msg::JointState cable_states;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cables_sub;
    rclcpp::Subscription<gazebo_msgs::msg::LinkState>::SharedPtr platform_sub;

    // subscriber to desired pose
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr setpoint_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desiredVel_sub, desiredAcc_sub;

    // publisher to tensions
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tensions_pub;
    sensor_msgs::msg::JointState tensions_msg;

    // pf pose and velocity
    vpHomogeneousMatrix M, M_, Md_;
    vpColVector v_, v_d, a_d;

    // model data
    double fMin, fMax;
    vpMatrix inertia_;
    std::vector<vpTranslationVector> Pf, Pp;
    unsigned int n_cable;

    TDA tda;


    // callback for platform state
    void PFState_cb(gazebo_msgs::msg::LinkState::ConstSharedPtr _msg)
    {
        platform_ok = true;
        M_.insert(vpTranslationVector(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z));
        M_.insert(vpQuaternionVector(_msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z,_msg->pose.orientation.w));
        v_.resize(6);
        v_[0]=_msg->twist.linear.x; v_[1]=_msg->twist.linear.y; v_[2]=_msg->twist.linear.z;
        v_[3]=_msg->twist.angular.x;  v_[4]=_msg->twist.angular.y; v_[5]=_msg->twist.angular.z;
    }

    // callback for pose setpoint
    void Setpoint_cb(geometry_msgs::msg::Pose::ConstSharedPtr _msg)
    {
        Md_.insert(vpTranslationVector(_msg->position.x, _msg->position.y, _msg->position.z));
        Md_.insert(vpQuaternionVector(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z, _msg->orientation.w));
    }

    // callback for cable states
    void Cables_cb(sensor_msgs::msg::JointState::ConstSharedPtr _msg)
    {
        cables_ok = true;
        cable_states = *_msg;
    }

    void DesiredVel_cb(geometry_msgs::msg::Twist::ConstSharedPtr _msg)
    {
        trajectory_ok = true;
        v_d.resize(6);
        v_d[0]=_msg->linear.x; v_d[1]=_msg->linear.y; v_d[2]=_msg->linear.z;
        v_d[3]=_msg->angular.x; v_d[4]=_msg->angular.y; v_d[5]=_msg->angular.z;
    }

    void DesiredAcc_cb(geometry_msgs::msg::Twist::ConstSharedPtr _msg)
    {
        a_d.resize(6);
        a_d[0]=_msg->linear.x; a_d[1]=_msg->linear.y; a_d[2]=_msg->linear.z;
        a_d[3]=_msg->angular.x; a_d[4]=_msg->angular.y; a_d[5]=_msg->angular.z;
    }
};

#endif //CABLE_ROBOT_GCU_H
