#include "youbot_motion_controller.hpp"

YouBotMotionController::YouBotMotionController(ros::NodeHandle &private_node_handle) : nh_(private_node_handle) {
    sub_config_ = nh_.subscribe("/youbot/control", 1, &YouBotMotionController::control_callback, this);
    pub_base_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pub_arm_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_1/arm_controller/command", 100);
    base_position_client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    orientation_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    arm_position_client_ = nh_.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
}

void YouBotMotionController::control_callback(const youbot_msgs::Control &control_msg) {
    Eigen::VectorXf configuration(8);
    configuration(0) = control_msg.X;
    configuration(1) = control_msg.Y;
    configuration(2) = control_msg.theta;
    configuration(3) = control_msg.joint_1;
    configuration(4) = control_msg.joint_2;
    configuration(5) = control_msg.joint_3;
    configuration(6) = control_msg.joint_4;
    configuration(7) = control_msg.joint_5;
    config_to_go_.push(configuration);
}

Eigen::VectorXf YouBotMotionController::get_current_configuration(){
    Eigen::VectorXf configuration(8);
    gazebo_msgs::GetLinkState srv_base;
    gazebo_msgs::GetModelState srv_orientation;
    gazebo_msgs::GetJointProperties srv_joint;
    srv_base.request.link_name = "youbot::base_footprint";
    base_position_client_.call(srv_base);
    configuration(0) = srv_base.response.link_state.pose.position.x;
    configuration(1) = srv_base.response.link_state.pose.position.y;
    srv_orientation.request.model_name = "youbot";
    orientation_client_.call(srv_orientation);
    configuration(2) = srv_orientation.response.pose.orientation.z * M_PI;
    for (int i=1; i<6; i++){
        srv_joint.request.joint_name = "youbot::arm_joint_" + std::to_string(i);
        arm_position_client_.call(srv_joint);
        configuration(2+i) = srv_joint.response.position.front();
    }
    return configuration;
}