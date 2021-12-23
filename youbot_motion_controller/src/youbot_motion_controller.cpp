#include "youbot_motion_controller.hpp"

YouBotMotionController::YouBotMotionController(ros::NodeHandle &private_node_handle) : nh_(private_node_handle) {
    sub_config_ = nh_.subscribe("/youbot/control", 1, &YouBotMotionController::control_callback, this);
    pub_base_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pub_arm_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_1/arm_controller/command", 100);
    base_position_client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    arm_position_client_ = nh_.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

    l = 0.38573, r = 0.05, w_max = 5250 * 2 * M_PI / 60 / 26;
    w_T_v.resize(4, 3);
    w_T_v << 1, -1, -l,
            1, 1, l,
            1, 1, -l,
            1, -1, l;
    w_T_v *= 1 / r;
    v_T_w.resize(3, 4);
    v_T_w << 1, 1, 1, 1,
            -1, 1, 1, -1,
            -1 / l, 1 / l, -1 / l, 1 / l;
    v_T_w *= r / 4;
    joint_velocity.resize(5);
    joint_velocity << 0.75, 0.27, 0.6, 0.45, 0.55;

    arm_on_its_way = false;
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
    Eigen::Vector3f direction(v_x, v_y, v_angular);
    Eigen::Vector4f w = w_T_v * direction;
    float current_max = w.cwiseAbs().maxCoeff();
    if (current_max > w_max) {
        w *= w_max / current_max;
        direction = v_T_w * w;
    }
    base_msg.linear.x = direction.x();
    base_msg.linear.y = direction.y();
    base_msg.angular.z = direction.z();
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
        arm_on_its_way = false;
        return;
    }

    Eigen::Rotation2Df t(config_current(2));
    Eigen::Vector2f distance = config_goal.head<2>() - config_current.head<2>();
    Eigen::Vector2f direction = t.inverse() * distance * kp_xy;
    float v_angular = kp_theta * (config_goal(2) - config_current(2));

    move_base_direction(direction(0), direction(1), v_angular);
    if (!arm_on_its_way){
        arm_on_its_way = true;
        move_arm(config_goal.tail(5), ros::Duration(0.5));
    }
}

Eigen::VectorXf YouBotMotionController::travel_time(const Eigen::VectorXf &config1, const Eigen::VectorXf &config2) {
    Eigen::VectorXf configuration = config2 - config1;
    // Compute max velocity of the base
    Eigen::Rotation2Df t(config1(2));
    Eigen::Vector2f distance = t * configuration.head<2>();
    Eigen::Vector3f v_base(distance(0), distance(1), configuration(2));
    Eigen::Vector4f w = w_T_v * v_base;
    w *= w_max / w.cwiseAbs().maxCoeff();
    v_base = v_T_w * w;

    Eigen::VectorXf velocity(8);
    velocity << v_base, joint_velocity;
    Eigen::VectorXf travel_time = (configuration.array() / velocity.array()).cwiseAbs();
    for (int i = 0; i < 8; i++) {
        travel_time(i) = std::isnan(travel_time(i)) ? 0.0 : travel_time(i);
    }
    return travel_time;
}
