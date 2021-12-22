#include "youbot_motion_controller.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_motion_controller_node");
    ros::NodeHandle node_handle;
    YouBotMotionController ymc(node_handle);
    ros::Rate rate(50);

    while (ros::ok()) {
        ymc.process_configs();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
