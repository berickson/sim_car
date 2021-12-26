#ifndef CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_
#define CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <mx_joint_controller_msgs/msg/joint_command.hpp>

namespace car_gazebo_plugin
{

class CarGazeboPlugin : public gazebo::ModelPlugin
{
public:
  CarGazeboPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  void Update();

  using JointState = sensor_msgs::msg::JointState;
  // using JointCommand = mx_joint_controller_msgs::msg::JointCommand;

  std::string robot_namespace_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;

  gazebo::common::Time last_sim_time_;
  gazebo::common::Time last_update_time_;
  double update_period_ms_;

  gazebo::event::ConnectionPtr update_connection_;

  std::map<std::string, std::pair<gazebo::physics::JointPtr, gazebo::common::PID>> joints_;
  std::map<std::string, double> joint_targets_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
  // rclcpp::Subscription<JointCommand>::SharedPtr joint_command_sub_;
};

}

#endif  // CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_