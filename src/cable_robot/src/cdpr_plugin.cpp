#include "cable_robot/cdpr_plugin.h"

namespace gazebo
{
    void CDPRPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model_ = _model;

        // register ROS node & time
        rosnode_ = gazebo_ros::Node::Get(_sdf);
        RCLCPP_INFO(rosnode_->get_logger(), "Loading CDPR Gazebo Plugin");

        // get cable joints
        std::string name;
        for (auto &joint: model_->GetJoints())
        {
            name = joint->GetName();
            if (name.find("cable") == 0) // we actuate only prismatic joints: cable{i}_joint
            {
                cableJoints_.push_back(joint);
                // save name
                cableJointsState_.name.push_back(name);
                // get maximum effort
                fMax = joint->GetEffortLimit(0);
            }
        }
        cableJointsState_.position.resize(cableJoints_.size());
        cableJointsState_.velocity.resize(cableJoints_.size());
        cableJointsState_.effort.resize(cableJoints_.size());

        // get frame and platform links
        for (auto &link: model_->GetLinks())
        {
            if (link->GetName() == "frame")
                frameLink_ = link;
            else if (link->GetName() == "platform")
                platformLink_ = link;
        }
        platformState_.link_name = "platform";
        platformState_.reference_frame = "frame";

        // Init publishers & subscribers
        cableControlSub_ = rosnode_->create_subscription<sensor_msgs::msg::JointState>(
                "cable_control", 5,
                std::bind(&CDPRPlugin::cableControlCallback, this, std::placeholders::_1)
        );
        cableJointsStatePub_ = rosnode_->create_publisher<sensor_msgs::msg::JointState>("cable_joints_state", 5);
        platformStatePub_ = rosnode_->create_publisher<gazebo_msgs::msg::LinkState>("platform_state", 5);

        // Register plugin update
        updateEvent_ = event::Events::ConnectWorldUpdateBegin(std::bind(&CDPRPlugin::Update, this));
        RCLCPP_INFO(rosnode_->get_logger(), "Started CDPR Plugin for %s.", _model->GetName().c_str());
    }

    void CDPRPlugin::Update()
    {
        // set cable joint forces
        if (controlReceived_)
        {
            for (unsigned int i = 0; i < cableControlCmd_.name.size(); ++i)
            {
                if (cableControlCmd_.effort[i] > 0)
                    cableJoints_[i]->SetForce(0, std::min(cableControlCmd_.effort[i], fMax));
            }
        }

        // publish joint states
        cableJointsState_.header.stamp = rosnode_->now();
        for (unsigned int i = 0; i < cableJoints_.size(); ++i)
        {
            cableJointsState_.position[i] = cableJoints_[i]->Position();
            cableJointsState_.velocity[i] = cableJoints_[i]->GetVelocity(0);
            cableJointsState_.effort[i] = cableJoints_[i]->GetForce(0);
        }
        cableJointsStatePub_->publish(cableJointsState_);

        // publish platform state
        auto pf_pose = platformLink_->WorldPose() - frameLink_->WorldPose();
        platformState_.pose.position.x = pf_pose.Pos().X();
        platformState_.pose.position.y = pf_pose.Pos().Y();
        platformState_.pose.position.z = pf_pose.Pos().Z();
        platformState_.pose.orientation.x = pf_pose.Rot().X();
        platformState_.pose.orientation.y = pf_pose.Rot().Y();
        platformState_.pose.orientation.z = pf_pose.Rot().Z();
        platformState_.pose.orientation.w = pf_pose.Rot().W();
        auto vel = pf_pose.Rot().RotateVector(platformLink_->RelativeLinearVel());
        platformState_.twist.linear.x = vel.X();
        platformState_.twist.linear.y = vel.Y();
        platformState_.twist.linear.z = vel.Z();
        vel = pf_pose.Rot().RotateVector(platformLink_->RelativeAngularVel());
        platformState_.twist.angular.x = vel.X();
        platformState_.twist.angular.y = vel.Y();
        platformState_.twist.angular.z = vel.Z();

        platformStatePub_->publish(platformState_);
    }
}   // namespace gazebo
