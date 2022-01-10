#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <ros/ros.h>
#include <youbot_msgs/Control.h>
#include <sys/stat.h>

#define MAXBUFSIZE  ((int) 1e6)

inline bool file_exists(const std::string &name) {
    // Source: https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-14-17-c
    struct stat buffer{};
    return (stat(name.c_str(), &buffer) == 0);
}


Eigen::MatrixXd readMatrix(const char *filename) {
    // Source: https://stackoverflow.com/questions/20786220/eigen-library-initialize-matrix-with-data-from-file-or-existing-stdvector
    if (!file_exists(filename)) {
        ROS_INFO_STREAM("FileNotFound");
        Eigen::MatrixXd result;
        return result;
    }
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(filename);
    while (!infile.eof()) {
        std::string line;
        getline(infile, line);
        int temp_cols = 0;
        std::stringstream stream(line);
        while (!stream.eof())
            stream >> buff[cols * rows + temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();

    rows--;

    // Populate matrix with numbers.
    Eigen::MatrixXd result(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i, j) = buff[cols * i + j];

    return result;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_configuration_reader");
    ros::NodeHandle node_handle;
    ros::Publisher pub = node_handle.advertise<youbot_msgs::Control>("/youbot/control", 1000);
    std::string path_str;
    if (ros::param::has("/youbot/configurations_path")) {
        ros::param::get("/youbot/configurations_path", path_str);
    } else {
        ROS_INFO_STREAM("rosparam /youbot/configurations_path not set");
        return 0;
    }
    ROS_INFO_STREAM("Reading: " << path_str);
    Eigen::MatrixXd mat = readMatrix(path_str.c_str());
    ros::spinOnce();

    int i = 0;
    Eigen::VectorXd configuration;

    ROS_INFO_STREAM("Number of configurations: " << mat.rows());
    ros::Rate rate(10);
    while (ros::ok() && i < mat.rows()) {
        configuration = mat.row(i);
        youbot_msgs::Control msg;
        msg.X = configuration(0);
        msg.Y = configuration(1);
        msg.theta = configuration(2);
        msg.joint_1 = configuration(3);
        msg.joint_2 = configuration(4);
        msg.joint_3 = configuration(5);
        msg.joint_4 = configuration(6);
        msg.joint_5 = 0;
        pub.publish(msg);
        i++;
        rate.sleep();
    }
    return 0;
}