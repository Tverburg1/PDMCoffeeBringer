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

# define MY_PI 3.141592653589793238462643383279502884

class YouBotMotionController {
public:
    explicit YouBotMotionController(ros::NodeHandle &private_node_handle);
    std::queue<Eigen::VectorXf> config_to_go_;
    ros::Publisher pub_base_;
    ros::Publisher pub_arm_;
    Eigen::VectorXf get_current_configuration();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_config_;
    ros::ServiceClient base_position_client_;
    ros::ServiceClient arm_position_client_;
    ros::ServiceClient orientation_client_;

    void control_callback(const youbot_msgs::Control &control_msg);
};