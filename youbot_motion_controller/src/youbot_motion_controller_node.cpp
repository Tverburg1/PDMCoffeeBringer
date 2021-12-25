#include "youbot_motion_controller.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_motion_controller_node");
    ros::NodeHandle node_handle;
    float frequency = 50;
    YouBotMotionController ymc(node_handle, 1 / frequency);
    ros::Rate rate(frequency);

    while (ros::ok()) {
        ymc.process_configs();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
