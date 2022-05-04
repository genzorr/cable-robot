#include "cable_robot/cdpr_plugin.h"

namespace gazebo
{
    void CDPRPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model = _model;

        // register ROS node & time
        rosnode = gazebo_ros::Node::Get(_sdf);
        RCLCPP_INFO(rosnode->get_logger(), "Loading CDPR Gazebo Plugin");

        // get cable joints
        std::string name;
        for (auto &joint: model->GetJoints())
        {
            name = joint->GetName();
            if (name.find("cable") == 0) // we actuate only prismatic joints: cable{i}_joint
            {
                cableJoints.push_back(joint);
                // get maximum effort
                fMax = joint->GetEffortLimit(0);
            }
        }

        // get frame and platform links
        for (auto &link: model->GetLinks())
        {
            if (link->GetName() == "frame")
                frameLink = link;
            else if (link->GetName() == "platform")
                platformLink = link;
        }
        platformState.link_name = "platform";
        platformState.reference_frame = "frame";

        // Init publishers & subscribers
        tensionsSub = rosnode->create_subscription<sensor_msgs::msg::JointState>(
                "tensions", 5,
                std::bind(&CDPRPlugin::cableControlCallback, this, std::placeholders::_1)
        );
        platformStatePub = rosnode->create_publisher<gazebo_msgs::msg::LinkState>("platform_state", 5);

        // Register plugin update
        updateEvent = event::Events::ConnectWorldUpdateBegin(std::bind(&CDPRPlugin::Update, this));
        RCLCPP_INFO(rosnode->get_logger(), "Started CDPR Plugin for %s.", _model->GetName().c_str());
    }

    void CDPRPlugin::Update()
    {
        // set cable joint forces
        if (controlReceived)
        {
            for (unsigned int i = 0; i < tensionsCmd.name.size(); ++i)
            {
                if (tensionsCmd.effort[i] > 0)
                    cableJoints[i]->SetForce(0, std::min(tensionsCmd.effort[i], fMax));
            }
        }

        // publish platform state
        auto pf_pose = platformLink->WorldPose() - frameLink->WorldPose();
        platformState.pose.position.x = pf_pose.Pos().X();
        platformState.pose.position.y = pf_pose.Pos().Y();
        platformState.pose.position.z = pf_pose.Pos().Z();
        platformState.pose.orientation.x = pf_pose.Rot().X();
        platformState.pose.orientation.y = pf_pose.Rot().Y();
        platformState.pose.orientation.z = pf_pose.Rot().Z();
        platformState.pose.orientation.w = pf_pose.Rot().W();
        auto vel = pf_pose.Rot().RotateVector(platformLink->RelativeLinearVel());
        platformState.twist.linear.x = vel.X();
        platformState.twist.linear.y = vel.Y();
        platformState.twist.linear.z = vel.Z();
        vel = pf_pose.Rot().RotateVector(platformLink->RelativeAngularVel());
        platformState.twist.angular.x = vel.X();
        platformState.twist.angular.y = vel.Y();
        platformState.twist.angular.z = vel.Z();

        platformStatePub->publish(platformState);
    }
}   // namespace gazebo
