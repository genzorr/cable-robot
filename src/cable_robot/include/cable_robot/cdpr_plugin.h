#ifndef CDPR_PLUGIN_H
#define CDPR_PLUGIN_H

#include "cable_robot_interfaces/msg/tensions.hpp"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_msgs/msg/link_state.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
//#include "cable_robot/msg/tensions.hpp"

namespace gazebo {
class CDPRPlugin : public ModelPlugin {
  struct Tension {
    ignition::math::Vector3d force;
    ignition::math::Vector3d point;
    std::string name;
  };

public:
  CDPRPlugin() : fMax(0.), controlReceived(false), cableJoints(){};

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  virtual void Update();

private:
  void cableControlCallback(sensor_msgs::msg::JointState::ConstSharedPtr _msg) {
    tensionsCmd = *_msg;
    controlReceived = true;
  }

private:
  double fMax;
  bool controlReceived;

  gazebo_ros::Node::SharedPtr rosnode;
  physics::ModelPtr model;
  event::ConnectionPtr updateEvent;

  std::vector<physics::JointPtr> cableJoints; // gazebo cable prismatic joints
  sensor_msgs::msg::JointState
      tensionsCmd; // command received from ROS to control cables

  gazebo_msgs::msg::LinkState platformState; // spatial state of platfrom
  geometry_msgs::msg::Accel platformAccel;
  physics::LinkPtr frameLink, platformLink; // frame and platform Gazebo links

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr tensionsSub;
  rclcpp::Publisher<gazebo_msgs::msg::LinkState>::SharedPtr platformStatePub;
  rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr platformAccelPub;
};

GZ_REGISTER_MODEL_PLUGIN(CDPRPlugin)
} // namespace gazebo
#endif // CDPR_PLUGIN_H