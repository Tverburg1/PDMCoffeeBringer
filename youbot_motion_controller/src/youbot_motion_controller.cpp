#include "youbot_motion_controller.hpp"

YouBotMotionController::YouBotMotionController(ros::NodeHandle &private_node_handle, float dt) : nh_(
        private_node_handle), dt(dt) {
    sub_config_ = nh_.subscribe("/youbot/control", 1, &YouBotMotionController::control_callback, this);
    sub_pid_ = nh_.subscribe("/youbot/base/set_controller", 1, &YouBotMotionController::set_controller_callback, this);
    pub_base_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pub_arm_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_1/arm_controller/command", 100);
    base_position_client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    arm_position_client_ = nh_.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

    l = 0.38573, r = 0.05, w_max = 5250 * 2 * M_PI / 60 / 26;
    w_J_c.resize(4, 3);
    w_J_c << 1, -1, -l,
            1, 1, l,
            1, 1, -l,
            1, -1, l;
    w_J_c *= 1 / r;
    c_J_w.resize(3, 4);
    c_J_w << 1, 1, 1, 1,
            -1, 1, 1, -1,
            -1 / l, 1 / l, -1 / l, 1 / l;
    c_J_w *= r / 4;
    joint_velocity.resize(5);
    joint_velocity << 0.75, 0.27, 0.6, 0.45, 0.55;

    m_tot = 32, I_tot = 4.3, I_wheel = 0.015218160428, T_max = 2;
    mass_array << m_tot, m_tot, I_tot;
    v_current.setZero();

    arm_on_its_way = false;
}

void YouBotMotionController::set_controller_callback(const youbot_msgs::ControlSettings &msg) {
    k_pos = msg.k_pos;
    k_vel = msg.k_vel;
    k_exp_vel = msg.k_exp_vel;
    v_current.setZero();
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

void YouBotMotionController::move_base_towards_position(Eigen::Vector3f position) {
    Eigen::Vector3f current_pos = get_current_configuration().head<3>();
    Eigen::Vector3f dist = position - current_pos;
    Eigen::AngleAxis<float> t(current_pos(2), Eigen::Vector3f(0, 0, 1));
//    F = k_pos * x - k_vel * e^{-k_exp_vel * |x|} * v
    Eigen::Vector3f force = t.inverse() * dist * k_pos - k_vel * std::exp(-k_exp_vel * dist.norm()) * v_current;
    apply_force_on_base(force);
}

void YouBotMotionController::apply_force_on_base(Eigen::Vector3f &force) {
    geometry_msgs::Twist base_msg;
    // Check domain of input velocity
    v_current = get_base_velocity_from_force(force);
    base_msg.linear.x = v_current.x();
    base_msg.linear.y = v_current.y();
    base_msg.angular.z = v_current.z();
    pub_base_.publish(base_msg);
}

Eigen::Vector3f YouBotMotionController::get_base_velocity_from_force(Eigen::Vector3f &force) const {
    float w_acc_max = T_max / I_wheel;
    // Calculate new velocity with a small time step dt
    Eigen::Vector4f w_acc = w_J_c * (force.array() / mass_array).matrix();
    float current_max = w_acc.cwiseAbs().maxCoeff();
    if (current_max > w_acc_max) {
        w_acc *= w_acc_max / current_max;
    }
    Eigen::Vector3f v_new = v_current.array() + (c_J_w * w_acc).array() * dt;
    // Limit velocity so it is lower than the maximal velocity
    Eigen::Vector4f w = w_J_c * v_new;
    current_max = w.cwiseAbs().maxCoeff();
    if (current_max > w_max) {
        w *= w_max / current_max;
        v_new = c_J_w * w;
    }
    return v_new;
}

void YouBotMotionController::brake() {
    if ((v_current.array() == 0).all()) {
        return;
    }
    Eigen::Vector3f F_opposite = -1000 * v_current;
    Eigen::Vector3f v_min = get_base_velocity_from_force(F_opposite);
    if ((v_current.cwiseProduct(v_min).array() < 0).all()) {
        geometry_msgs::Twist base_msg;
        pub_base_.publish(base_msg);
        v_current.setZero();
    } else {
        move_base_towards_position(previous_config_.head<3>());
    }
}

void YouBotMotionController::process_configs() {
    if (config_to_go_.empty()) {
        brake();
        return;
    }

    Eigen::VectorXf config_goal = config_to_go_.front();
    Eigen::VectorXf config_current = get_current_configuration();

    if ((config_goal - config_current).norm() < 0.01) { // Configuration reached
        previous_config_ = config_goal;
        config_to_go_.pop();
        arm_on_its_way = false;
        return;
    }

    move_base_towards_position(config_goal.head<3>());

    if (!arm_on_its_way) {
        arm_on_its_way = true;
        move_arm(config_goal.tail(5), ros::Duration(0.5));
    }
}

Eigen::VectorXf
YouBotMotionController::get_travel_time(const Eigen::VectorXf &config1, const Eigen::VectorXf &config2) {
    Eigen::VectorXf configuration = config2 - config1;
    // Compute max velocity of the base
    Eigen::Rotation2Df t(config1(2));
    Eigen::Vector2f distance = t * configuration.head<2>();
    Eigen::Vector3f v_base(distance(0), distance(1), configuration(2));
    Eigen::Vector4f w = w_J_c * v_base;
    w *= w_max / w.cwiseAbs().maxCoeff();
    v_base = c_J_w * w;

    Eigen::VectorXf velocity(8);
    velocity << v_base, joint_velocity;
    Eigen::VectorXf travel_time = (configuration.array() / velocity.array()).cwiseAbs();
    for (int i = 0; i < 8; i++) {
        travel_time(i) = std::isnan(travel_time(i)) ? 0.0 : travel_time(i);
    }
    return travel_time;
}
