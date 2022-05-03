#ifndef CDPR_PLUGIN_H
#define CDPR_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo_msgs/msg/link_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "../../../build/cable_robot/rosidl_generator_cpp/cable_robot/msg/tensions.hpp"
#include "../../../build/cable_robot/rosidl_generator_cpp/cable_robot/msg/detail/tensions__struct.hpp"
//#include "cable_robot/msg/tensions.hpp"

namespace gazebo
{
    class CDPRPlugin : public ModelPlugin
    {
        struct Tension
        {
            ignition::math::Vector3d force;
            ignition::math::Vector3d point;
            std::string name;
        };

    public:
        CDPRPlugin() : fMax(0.), controlReceived_(false), cableJoints_() {};
        ~CDPRPlugin() override = default;

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
        virtual void Update();

    private:
        void cableControlCallback(sensor_msgs::msg::JointState::ConstSharedPtr _msg)
        {
            cableControlCmd_ = *_msg;
            controlReceived_ = true;
        }

    private:
        double fMax;
        bool controlReceived_;

        gazebo_ros::Node::SharedPtr rosnode_;
        physics::ModelPtr model_;
        event::ConnectionPtr updateEvent_;

        std::vector<physics::JointPtr> cableJoints_;    // gazebo cable prismatic joints
        sensor_msgs::msg::JointState cableJointsState_; // state of cable joints command sent to ROS
        sensor_msgs::msg::JointState cableControlCmd_;  // command received from ROS to control cables

        gazebo_msgs::msg::LinkState platformState_;     // spatial state of platfrom
        physics::LinkPtr frameLink_, platformLink_;     // frame and platform Gazebo links

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cableControlSub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cableJointsStatePub_;
        rclcpp::Publisher<gazebo_msgs::msg::LinkState>::SharedPtr platformStatePub_;
    };

    GZ_REGISTER_MODEL_PLUGIN(CDPRPlugin)
}
#endif // CDPR_PLUGIN_H