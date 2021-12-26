#include "car_gazebo_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <iostream>
#include <gazebo_ros/node.hpp>

namespace car_gazebo_plugin
{

CarGazeboPlugin::CarGazeboPlugin()
: robot_namespace_{""},
  last_sim_time_{0},
  last_update_time_{0},
  update_period_ms_{8}
{
}

void CarGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get model and world references
  model_ = model;
  world_ = model_->GetWorld();
  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("friction_model", std::string{"cone_model"});

  if (sdf->HasElement("robotNamespace")) {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  // Set up ROS node and subscribers and publishers
  ros_node_ = gazebo_ros::Node::Get(sdf);
  RCLCPP_INFO(ros_node_->get_logger(), "Loading Car Gazebo Plugin");

  rclcpp::QoS qos(10);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  joint_state_pub_ = ros_node_->create_publisher<JointState>("/joint_states", qos);
  // joint_command_sub_ = ros_node_->create_subscription<JointCommand>(
  //   "/cm730/joint_commands",
  //   10,
  //   [ = ](JointCommand::SharedPtr cmd) {
  //     for (size_t i = 0; i < cmd->name.size(); ++i) {
  //       joint_targets_[cmd->name[i]] = cmd->position[i];
  //     }
  //   }
  // );

  // Find joints
  auto allJoints = model_->GetJoints();
  for (auto const & j : allJoints) {
    if (j->GetType() == gazebo::physics::Joint::FIXED_JOINT) {
      continue;
    }

    auto pid = gazebo::common::PID{};
    pid.SetPGain(200.0);
    pid.SetIGain(0.0);
    pid.SetDGain(0.0);

    auto const & name = j->GetName();
    joints_[name] = std::make_pair(j, pid);
    joint_targets_[name] = 0.0;
  }

  RCLCPP_DEBUG(ros_node_->get_logger(), "Got joints:");
  for (auto const & j : joints_) {
    RCLCPP_DEBUG(ros_node_->get_logger(), j.first.c_str());
  }

  // Hook into simulation update loop
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&CarGazeboPlugin::Update, this));
}

void CarGazeboPlugin::Update()
{
  auto cur_time = world_->SimTime();
  if (last_sim_time_ == 0) {
    last_sim_time_ = cur_time;
    last_update_time_ = cur_time;
    return;
  }

  auto dt = (cur_time - last_sim_time_).Double();

  // Publish joint states
  auto update_dt = (cur_time - last_update_time_).Double();
  if (update_dt * 1000 >= update_period_ms_) {
    auto msg = JointState{};
    msg.header.stamp = ros_node_->now();

    for (auto & j : joints_) {
      auto const & name = j.first;
      auto & joint = j.second.first;
      auto position = joint->Position();
      msg.name.push_back(name);
      msg.position.push_back(position);
    }
    joint_state_pub_->publish(msg);
    last_update_time_ = cur_time;
  }

  // return;
  // // Update joint PIDs
  // for (auto & j : joints_) {
  //   auto & joint = j.second.first;
  //   auto & pid = j.second.second;

  //   auto error = joint->Position() - joint_targets_[j.first];

  //   auto force = pid.Update(error, dt);
  //   joint->SetForce(0, force);
  // }

  last_sim_time_ = cur_time;
}

GZ_REGISTER_MODEL_PLUGIN(CarGazeboPlugin)

}  // namespace Car_gazebo_plugin