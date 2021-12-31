#include "youbot_motion_controller.hpp"


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "youbot_configuration_publisher_node");
    ros::NodeHandle node_handle("~");
    ros::Publisher pub = node_handle.advertise<youbot_msgs::Control>("/youbot/current_configuration", 100);
    float frequency = 20;
    YouBotMotionController ymc(node_handle, 1 / frequency);
    ros::Rate rate(frequency);
    while (ros::ok()) {
        youbot_msgs::Control msg;
        auto config = ymc.get_current_configuration();
        msg.X = config(0);
        msg.Y = config(1);
        msg.theta = config(2);
        msg.joint_1 = config(3);
        msg.joint_2 = config(4);
        msg.joint_3 = config(5);
        msg.joint_4 = config(6);
        msg.joint_5 = config(7);
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}