#include "youbot_motion_controller.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_motion_controller_node");
    ros::NodeHandle node_handle;
    YouBotMotionController ymc(node_handle);
    int f = 50;
    ros::Rate rate(f);
    float kp_xy = 10;
    float kp_theta = 10;

    while (ros::ok()) {
        if (ymc.config_to_go_.empty()) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        Eigen::VectorXf config_goal = ymc.config_to_go_.front();
        Eigen::VectorXf config_current = ymc.get_current_configuration();
        trajectory_msgs::JointTrajectory arm_msg;
        geometry_msgs::Twist base_msg;
        if ((config_goal - config_current).norm() < 0.1) {
            ymc.pub_base_.publish(base_msg); // stop base
            ymc.config_to_go_.pop();
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        Eigen::Rotation2Df t(config_current(2));
        Eigen::Vector2f distance = config_goal.head<2>() - config_current.head<2>();
        Eigen::Vector2f direction = t.inverse() * distance * kp_xy;
        if (direction.norm() > 1) {
            direction.normalize();
        }
        base_msg.linear.x = direction(0);
        base_msg.linear.y = direction(1);
        float v_angular = kp_theta * (config_goal(2) - config_current(2));
        if (v_angular>1) {
            v_angular = 1;
        } else if (v_angular < -1){
            v_angular = -1;
        }
        base_msg.angular.z = v_angular;


        trajectory_msgs::JointTrajectoryPoint point;
        for (int i = 3; i < 8; i++) {
            point.positions.push_back(config_goal(i));
        }
        point.time_from_start = ros::Duration(0.5);
        arm_msg.points.push_back(point);
        for (int i = 0; i < 5; i++) {
            std::stringstream jointName;
            jointName << "arm_joint_" << (i + 1);
            arm_msg.joint_names.push_back(jointName.str());
        }
        arm_msg.header.frame_id = "arm_link_0";
        arm_msg.header.stamp = ros::Time::now();

        ymc.pub_base_.publish(base_msg);
        ymc.pub_arm_.publish(arm_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
