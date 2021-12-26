#ifndef CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_
#define CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

// #include <mx_joint_controller_msgs/msg/joint_command.hpp>



double sign_of(double x) {
    if(x>0) {
        return 1;
    }
    if(x<0) {
        return -1;
    }
    return 0;
}

class BicycleModel {
    double wheelbase_length;
    double steer_angle;
public:
    BicycleModel(double wheelbase_length, double steer_angle = 0.0) {
        this->wheelbase_length = wheelbase_length;
        this->steer_angle = steer_angle;
    }
    
    void set_steer_angle(double steer_angle) {
        this->steer_angle = steer_angle;
    }
    
    void set_rear_curvature(double k_rear) {
        this->steer_angle = atan(k_rear * this->wheelbase_length);
    }
    
    double get_rear_curvature(){
        return tan(this->steer_angle) / this->wheelbase_length;
    }
    
    void set_front_curvature(double k_front){
        this->steer_angle = asin(this->wheelbase_length * k_front);
    }

    double get_front_curvature() {
        return sin(this->steer_angle) / this->wheelbase_length;
    }
    
    double get_steer_angle() {
        return this->steer_angle;
    }
    
    BicycleModel get_offset_bicycle(double delta_y){
        if(this->steer_angle == 0.0) {
            return BicycleModel(wheelbase_length, 0.0);
        }
        BicycleModel new_bike(this->wheelbase_length);
        new_bike.set_rear_curvature(1./(1./this->get_rear_curvature() - delta_y));
        return new_bike;
    }
};

class AckermannModel : public BicycleModel {
    double front_wheelbase_width;
public:
    AckermannModel(double wheelbase_length, double front_wheelbase_width, double  steer_angle = 0.0)
        : BicycleModel(wheelbase_length, steer_angle)
    {
        this->front_wheelbase_width = front_wheelbase_width;
    }

    BicycleModel get_left_bicycle() {
        return this->get_offset_bicycle(this->front_wheelbase_width / 2.);
    }

    BicycleModel get_right_bicycle() {
        return this->get_offset_bicycle(-1. * this->front_wheelbase_width / 2.);
    }
};


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



  ////// added from fake_car_sim
double steer = 0;
        double velocity = 0;
        const double wheelbase_length = 0.3429 ;
        const double front_wheelbase_width = 0.25;
        const double rear_wheelbase_width = 0.25;
        const double shock_p = 250;
        const double shock_d = 30;
        const double wheel_diameter = .112;
        const double max_speed = 3;
        AckermannModel car_model={0.25, 0.25};



        gazebo::physics::JointControllerPtr jc;
        gazebo::physics::JointPtr fl_str_joint;
        gazebo::physics::JointPtr fr_str_joint;
        gazebo::physics::JointPtr fl_axle_joint;
        gazebo::physics::JointPtr fr_axle_joint;
        gazebo::physics::JointPtr bl_axle_joint;
        gazebo::physics::JointPtr br_axle_joint;
        gazebo::physics::JointPtr fl_shock_joint;
        gazebo::physics::JointPtr fr_shock_joint;
        gazebo::physics::JointPtr bl_shock_joint;
        gazebo::physics::JointPtr br_shock_joint;

        gazebo::common::PID fl_pid, fr_pid, bl_pid, br_pid, fl_shock_pid, fr_shock_pid, bl_shock_pid, br_shock_pid;
        // std::unique_ptr<ros::NodeHandle> n;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr odo_fl_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr odo_fr_pub;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;
        

        // ros::Publisher odo_fl_pub, odo_fr_pub, ackermann_pub;

        // ros::CallbackQueue ros_queue;

        // std::thread ros_queue_thread;

        void publish_state(){
            const int ticks_per_revolution = 42;

            std_msgs::msg::Int32 odo_fl;
            odo_fl.data = (int) fl_axle_joint->Position() * ticks_per_revolution / (M_2_PI);
            odo_fl_pub->publish(odo_fl);

            std_msgs::msg::Int32 odo_fr;
            odo_fr.data = (int) fr_axle_joint->Position() * ticks_per_revolution / (M_2_PI);
            odo_fr_pub->publish(odo_fr);

        }

        void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
            ackermann_msgs::msg::AckermannDriveStamped ad;
            ad.drive.steering_angle = msg->axes[0];
            ad.drive.speed = pow(fabs(msg->axes[1]),2)*sign_of(msg->axes[1]) * max_speed;
            ackermann_pub->publish(ad);
        }

        void ackermann_callback(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
            car_model.set_steer_angle(msg->drive.steering_angle);

            jc->SetPositionTarget(
                fl_str_joint->GetScopedName(), car_model.get_left_bicycle().get_steer_angle());

            jc->SetPositionTarget(
                fr_str_joint->GetScopedName(), car_model.get_right_bicycle().get_steer_angle());

            double curvature = car_model.get_rear_curvature();
            double speed = msg->drive.speed;
            double meters_per_rev = M_PI * wheel_diameter;
            double revs_per_sec = speed/meters_per_rev;
            double rads_per_sec = 2*M_PI*revs_per_sec;
            if(curvature == 0){
                jc->SetVelocityTarget(
                    bl_axle_joint->GetScopedName(), rads_per_sec);
                jc->SetVelocityTarget(
                    br_axle_joint->GetScopedName(), rads_per_sec);
            } else {
                double radius = 1./curvature;
                double left_radius = radius - rear_wheelbase_width / 2.;
                double right_radius = radius + rear_wheelbase_width / 2.;

                jc->SetVelocityTarget(
                    bl_axle_joint->GetScopedName(), rads_per_sec*left_radius/radius);
                jc->SetVelocityTarget(
                    br_axle_joint->GetScopedName(), rads_per_sec*right_radius/radius);
            }
        }

        void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
            ackermann_msgs::msg::AckermannDriveStamped ad;
            ad.drive.speed = msg->linear.x;
        
            if (msg->linear.x == 0) {
                ad.drive.steering_angle = 0;
            }
            else {
                car_model.set_rear_curvature(msg->angular.z / msg->linear.x);
                ad.drive.steering_angle = car_model.get_steer_angle();
            }
            ackermann_pub->publish(ad);
        }

        // private: void QueueThread()
        // {
        //     ros::Rate loop_rate(100);

        //     while (n->ok())
        //     {
        //         ros_queue.callAvailable();

        //         publish_state();
        //         loop_rate.sleep();
        //     }
        // }
        
        // public: FakeCarPlugin() : ModelPlugin()
        // {
        //     ROS_INFO("FakeCarPlugin() : ModelPlugin()");
        // }

        private: gazebo::physics::JointPtr get_joint(const char * joint_name) {
            auto joint = model_->GetJoint(joint_name);
            if(joint.get()==0) {
                RCLCPP_ERROR(ros_node_->get_logger(),"Failed to get_joint %s", joint_name);
            }
            
            return joint;
        }


  //       void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  //       {
  //           if (!ros::isInitialized())
  //           {
  //               ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
  //                   << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
  //               return;
  //           }

  //           model = _model;
  //           ROS_INFO("Connected to model %s", model_->GetName().c_str());
            
  //           jc = model_->GetJointController();

  //           // front left str
  //           fl_pid = common::PID(1, 0, 0);
  //           fl_str_joint  = get_joint("front_left_wheel_steer_joint");

  //           jc->SetPositionPID(
  //               fl_str_joint->GetScopedName(), fl_pid);

  //           // front left shock
  //           fl_shock_pid = common::PID(shock_p, 0, shock_d);
  //           fl_shock_joint = get_joint("front_left_shock_joint");
  //           jc->SetPositionPID(fl_shock_joint->GetScopedName(), fl_shock_pid);
  //           jc->SetPositionTarget(fl_shock_joint->GetScopedName(), 0.0);

  //           // front right shock
  //           fr_shock_pid = common::PID(shock_p, 0, shock_d);
  //           fr_shock_joint = get_joint("front_right_shock_joint");
  //           jc->SetPositionPID(fr_shock_joint->GetScopedName(), fr_shock_pid);
  //           jc->SetPositionTarget(fr_shock_joint->GetScopedName(), 0.0);

  //           // back left shock
  //           bl_shock_pid = common::PID(shock_p, 0, shock_d);
  //           bl_shock_joint = get_joint("back_left_shock_joint");
  //           jc->SetPositionPID(bl_shock_joint->GetScopedName(), bl_shock_pid);
  //           jc->SetPositionTarget(bl_shock_joint->GetScopedName(), 0.0);

  //           // back right shock
  //           br_shock_pid = common::PID(shock_p, 0, shock_d);
  //           br_shock_joint = get_joint("back_right_shock_joint");
  //           jc->SetPositionPID(br_shock_joint->GetScopedName(), br_shock_pid);
  //           jc->SetPositionTarget(br_shock_joint->GetScopedName(), 0.0);

  //           // front right str            
  //           fr_pid = common::PID(1, 0, 0);
  //           fr_str_joint  = get_joint("front_right_wheel_steer_joint");
  //           jc->SetPositionPID(
  //               fr_str_joint->GetScopedName(), fr_pid);

  //           fl_axle_joint = get_joint("front_left_wheel_joint");
  //           fr_axle_joint = get_joint("front_right_wheel_joint");

  //           // back left speed
  //           bl_pid = common::PID(0.1, 0.01, 0.0);
  //           bl_axle_joint = get_joint("back_left_wheel_joint");
  //           jc->SetVelocityPID(
  //               bl_axle_joint->GetScopedName(), bl_pid);
            

  //           br_pid = common::PID(0.1, 0.01, 0.0);
  //           br_axle_joint = get_joint("back_right_wheel_joint");
  //           jc->SetVelocityPID(
  //               br_axle_joint->GetScopedName(), br_pid);
            


  //           if (!ros::isInitialized())
  //           {
  //               int argc = 0;
  //               char **argv = NULL;
  //               ros::init(argc, argv, "fake_car_plugin",
  //                   ros::init_options::NoSigintHandler);
  //           }

            
  //           n.reset(new ros::NodeHandle("fake_car_plugin"));

  //           // publish
  //           odo_fl_pub = ros_node_->advertise<std_msgs::Int32>("/" + model_->GetName() + "/odo_fl", 10);
  //           odo_fr_pub = ros_node_->advertise<std_msgs::Int32>("/" + model_->GetName() + "/odo_fr", 10);
  //           ackermann_pub = ros_node_->advertise<ackermann_msgs::AckermannDriveStamped>("/" + model_->GetName() + "/cmd_ackermann", 10);

  //           // subscribe
  //           joy_sub = ros_node_->subscribe<sensor_msgs::Joy>(
  //               "/joy",
  //               2,
  //               &CarGazeboPlugin::joy_callback, this);


  // // joint_command_sub_ = ros_node_->create_subscription<JointCommand>(
  // //   "/cm730/joint_commands",
  // //   10,
  // //   [ = ](JointCommand::SharedPtr cmd) {
  // //     for (size_t i = 0; i < cmd->name.size(); ++i) {
  // //       joint_targets_[cmd->name[i]] = cmd->position[i];
  // //     }
  // //   }

  //           ackermann_sub = ros_node_->create_subscription<ackermann_msgs::msg:: AckermannDriveStamped>(
  //               "/" + model_->GetName() + "/cmd_ackermann",
  //               2,
  //               std::bind(&CarGazeboPlugin::ackermann_callback, this));



  //           // ros_queue_thread = std::thread(std::bind(&FakeCarPlugin::QueueThread, this));
  //       }  
};

}

#endif  // CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_