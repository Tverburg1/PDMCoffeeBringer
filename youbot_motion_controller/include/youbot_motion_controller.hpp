#pragma once

#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <youbot_msgs/Control.h>
#include <youbot_msgs/BaseControlSettings.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <string>
#include <tf/tf.h>

# define MY_PI 3.141592653589793238462643383279502884

class YouBotMotionController {
public:
    explicit YouBotMotionController(ros::NodeHandle &private_node_handle, float dt);

    std::queue<Eigen::VectorXf> config_to_go_;
    Eigen::VectorXf previous_config_;
    ros::Publisher pub_base_;
    ros::Publisher pub_arm_;

    Eigen::VectorXf get_current_configuration();

    void move_arm(const Eigen::VectorXf &arm_configuration, const ros::Duration &duration = ros::Duration(0.5)) const;

    void apply_force_on_base(Eigen::Vector3f &force);

    void brake();

    void move_base_towards_position(Eigen::Vector3f position);

    void process_configs();

    Eigen::VectorXf get_travel_time(const Eigen::VectorXf &config1, const Eigen::VectorXf &config2);

//    Eigen::Matrix3f k_pos, k_vel, k_exp_vel;
    Eigen::Array3f brake_distance, k_pos, k_vel;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_config_;
    ros::Subscriber sub_pid_;
    ros::ServiceClient base_position_client_;
    ros::ServiceClient arm_position_client_;

    void control_callback(const youbot_msgs::Control &control_msg);

    void set_controller_callback(const youbot_msgs::BaseControlSettings &msg);

    Eigen::Vector3f get_base_velocity_from_force(Eigen::Vector3f &force) const;

    float l, r, w_max;
    float m_tot, I_tot, I_wheel, T_max, dt;
    Eigen::Array<float, 3, 1> mass_array;
    Eigen::MatrixXf w_J_c, c_J_w;
    Eigen::Vector3f v_current;
    Eigen::VectorXf joint_velocity;
    bool arm_on_its_way, base_velocity_is_zero;
};