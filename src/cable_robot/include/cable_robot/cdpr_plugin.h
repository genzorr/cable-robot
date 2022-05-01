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
//#include "cable_robot/msg/tensions.hpp"
#include "../../../build/cable_robot/rosidl_generator_cpp/cable_robot/msg/tensions.hpp"
#include "../../../build/cable_robot/rosidl_generator_cpp/cable_robot/msg/detail/tensions__struct.hpp"

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
        CDPRPlugin() {}
        ~CDPRPlugin() {}

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
        virtual void Update();

    private:

        // parse received joint command (joint states)
        void JointCommandCallBack(sensor_msgs::msg::JointState::ConstSharedPtr _msg)
        {
            // store received joint state
            joint_command_ = *_msg;
            command_received_ = true;
        }

        // full cable tensions
        void TensionCallBack(const cable_robot::msg::Tensions::ConstSharedPtr &msg)
        {
            if(msg->names.size() != msg->direction.size() ||
               msg->names.size() != msg->tensions.size())
            {
                RCLCPP_WARN(rosnode_->get_logger(), "Received inconsistent tension dimensions");
                return;
            }
            command_received_ = true;
            for(unsigned long i = 0; i < msg->names.size(); ++i)
            {
                // look for corresponding cable in command
                auto cable = std::find_if(tension_command_.begin(),
                                          tension_command_.end(),
                                          [&](Tension &t){return t.name == msg->names[i];});
                if(cable != tension_command_.end())
                {
                    cable->force.X() = msg->direction[i].x;
                    cable->force.Y() = msg->direction[i].y;
                    cable->force.Z() = msg->direction[i].z;
                    cable->force.Normalize();
                    cable->force *= std::max<double>(0, msg->tensions[i]);
                }
            }
        }

    private:
        // -- general data ----------------------------------------
        rclcpp::Node::SharedPtr rosnode_;
//        ros::CallbackQueue callback_queue_;
        physics::ModelPtr model_;
        event::ConnectionPtr update_event_;
        double update_T_;

        // -- joint control ----------------------------------------
        // model joint data
        std::vector<physics::JointPtr> joints_;
        double f_max;

        // subscriber
        bool sim_cables_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;
        sensor_msgs::msg::JointState joint_command_;
        std::vector<Tension> tension_command_;
        bool command_received_;


        // -- publishers ----------------------------------------

        // publisher to joint state
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        sensor_msgs::msg::JointState joint_states_;
        double t_prev_;

        // publisher of platform position
        rclcpp::Publisher<gazebo_msgs::msg::LinkState>::SharedPtr pf_publisher_;
        gazebo_msgs::msg::LinkState pf_state_;
        physics::LinkPtr frame_link_, platform_link_;
    };
    GZ_REGISTER_MODEL_PLUGIN(CDPRPlugin)
}
#endif // CDPR_PLUGIN_H