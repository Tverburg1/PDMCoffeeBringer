#ifndef collision_check_header
#define collision_check_header

#include"3d_coordinate.h"
#include<vector>
#include<Eigen/Dense>

// Function for checking whether a linear trajectory between 2 configurations
// (begin and end) is collision-free. 
// This function returns true when the trajectory would result in collision and 
// false when the path is collision-free.
bool check_collision_between_two_configurations(Eigen::VectorXf begin, Eigen::VectorXf end);

// This function checks for a single configuration of the youbot, whether the 
// youbot is in collision when it is in that configuration.
// Returns true when a collision is found and false if the configuration is
// collision-free.
bool check_collision_single_configuration(Eigen::VectorXf configuration);

// This function checks for all spheres that model the youbot whether they are 
// colliding with the floor (z = 0).
// Returns true when a collision is found and false otherwise
bool check_collision_with_floor(std::vector<coordinate_3d> youbot_spheres);

// This Function checks whether the youbot is in collision with itself.
// Returns true when a collision is found and false otherwise
bool check_self_collision(std::vector<coordinate_3d> youbot_spheres);

// This function checks whether any of the spheres that represent the youbot 
// are in collision with any of the points in the point cloud. 
// Returns true when a collision is found and false otherwise.
bool check_collision_with_pointcloud(std::vector<coordinate_3d> youbot_spheres);

// This function checks whether any of the points in the pointcloud are in 
// collision with the base (modeled as a cilinder).
// Returns true when a collision is found and false otherwise.
bool check_collision_base_with_pointcloud(coordinate_3d base, std::vector<coordinate_3d> relevant_points);

// This function calculates for a given configuration, where in the 3D workspace
// all the joints (and the base and arm_base) are.
// input configuration is {X, Y, theta, psi, alpha, beta, gamma} (order can be changed if needed)
// output coordinate vector is {center_base (height 0), center_arm_base, joint_1, joint_2, joint_3, endpoint}
std::vector<coordinate_3d> configuration_to_joint_positions(Eigen::VectorXf configuration);

// This function divides the youbot into 8 spheres, when the joint positions of the youbot are given
// Input (positions of): 
// {center_base (height 0), center_arm_base, joint_1, joint_2, joint_3, endpoint}
// Output (spheres with proper radius, representing): 
// {Base, Arm_base, lower_half_of_segment_1, upper_half_of_segment_1, lower_half_of_segment_2, upper_half_of_segment_2, lower_half_of_segment_3, upper_half_of_segment_3}
std::vector<coordinate_3d> divide_youbot_into_spheres(std::vector<coordinate_3d> joint_positions);

#endif
