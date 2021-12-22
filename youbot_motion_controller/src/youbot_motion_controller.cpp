#include "youbot_motion_controller.hpp"

YouBotMotionController::YouBotMotionController(ros::NodeHandle &private_node_handle) : nh_(private_node_handle) {
    sub_config_ = nh_.subscribe("/youbot/control", 1, &YouBotMotionController::control_callback, this);
    pub_base_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pub_arm_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_1/arm_controller/command", 100);
    base_position_client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
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

Eigen::VectorXf YouBotMotionController::get_current_configuration() {
    Eigen::VectorXf configuration(8);
    gazebo_msgs::GetLinkState srv_base;
    gazebo_msgs::GetModelState srv_orientation;
    gazebo_msgs::GetJointProperties srv_joint;
    srv_base.request.link_name = "youbot::base_footprint";
    base_position_client_.call(srv_base);
    configuration(0) = srv_base.response.link_state.pose.position.x;
    configuration(1) = srv_base.response.link_state.pose.position.y;
    configuration(2) = tf::getYaw(srv_base.response.link_state.pose.orientation);
    for (int i = 1; i < 6; i++) {
        srv_joint.request.joint_name = "youbot::arm_joint_" + std::to_string(i);
        arm_position_client_.call(srv_joint);
        configuration(2 + i) = srv_joint.response.position.front();
    }
    return configuration;
}

void YouBotMotionController::move_arm(const Eigen::VectorXf &arm_configuration, const ros::Duration &duration) const {
    trajectory_msgs::JointTrajectory arm_msg;
    trajectory_msgs::JointTrajectoryPoint point;
    arm_msg.header.frame_id = "arm_link_0";
    point.time_from_start = duration;
    for (int i = 0; i < 5; i++) {
        arm_msg.joint_names.push_back("arm_joint_" + std::to_string(i + 1));
        point.positions.push_back(arm_configuration(i));
    }
    arm_msg.points.push_back(point);
    arm_msg.header.stamp = ros::Time::now();
    pub_arm_.publish(arm_msg);
}

void YouBotMotionController::move_base_direction(float v_x, float v_y, float v_angular) const {
    geometry_msgs::Twist base_msg;
    // Check domain of input velocity
    Eigen::Vector2f direction(v_x, v_y);
    if (direction.norm() > 1) {
        direction.normalize();
    }
    base_msg.linear.x = direction(0);
    base_msg.linear.y = direction(1);
    if (v_angular > 1) {
        v_angular = 1;
    } else if (v_angular < -1) {
        v_angular = -1;
    }
    base_msg.linear.x = direction(0);
    base_msg.linear.y = direction(1);
    base_msg.angular.z = v_angular;
    pub_base_.publish(base_msg);
}

void YouBotMotionController::process_configs() {
    if (config_to_go_.empty()) {
        return;
    }
    geometry_msgs::Twist base_msg;

    Eigen::VectorXf config_goal = config_to_go_.front();
    Eigen::VectorXf config_current = get_current_configuration();

    if ((config_goal - config_current).norm() < 0.01) { // Configuration reached
        pub_base_.publish(base_msg); // stop base movement
        config_to_go_.pop();
        return;
    }

    Eigen::Rotation2Df t(config_current(2));
    Eigen::Vector2f distance = config_goal.head<2>() - config_current.head<2>();
    Eigen::Vector2f direction = t.inverse() * distance * kp_xy;
    float v_angular = kp_theta * (config_goal(2) - config_current(2));

    move_base_direction(direction(0), direction(1), v_angular);
    move_arm(config_goal.tail(5), ros::Duration(0.5));
}
