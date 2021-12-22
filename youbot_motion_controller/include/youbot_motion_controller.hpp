#pragma once

#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <youbot_msgs/Control.h>
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
    explicit YouBotMotionController(ros::NodeHandle &private_node_handle);

    std::queue<Eigen::VectorXf> config_to_go_;
    ros::Publisher pub_base_;
    ros::Publisher pub_arm_;

    Eigen::VectorXf get_current_configuration();

    void move_arm(const Eigen::VectorXf &arm_configuration, const ros::Duration &duration = ros::Duration(0.5)) const;

    void move_base_direction(float v_x = 0, float v_y = 0, float v_angular = 0) const;

    void process_configs();

    float kp_xy = 10;
    float kp_theta = 10;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_config_;
    ros::ServiceClient base_position_client_;
    ros::ServiceClient arm_position_client_;

    void control_callback(const youbot_msgs::Control &control_msg);

    float l, r, w_max;
    Eigen::MatrixXf w_T_v;
    Eigen::MatrixXf v_T_w;


};