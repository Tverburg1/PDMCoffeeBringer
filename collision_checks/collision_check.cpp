#include"3d_coordinate.h"
#include"collision_check.h"
#include<vector>
#include"load_pointcloud.h"
#include<math.h>
#include<cmath>
#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace std;
using namespace Eigen;


// Function for checking whether a linear trajectory between 2 configurations
// (begin and end) is collision-free. (by linearly sampling and checking each
// for each of those samples whether it is in collision or not.
// This function returns true if any of the samples is in collision (so when the
// path is not collision-free) and false when the path is collision-free.
bool check_collision_between_two_configurations(VectorXf begin, VectorXf end)
{
	// [1] derive joint position for begin & end configuration
	vector<coordinate_3d> joint_positions_begin = configuration_to_joint_positions(begin);
	vector<coordinate_3d> joint_positions_end = configuration_to_joint_positions(end);
	
	// [2] for all joints check how much it will displace from the begin config 
	// to end config and find the maxium amount any joint moves
	double max_displacement = 0.0;
	double displacement;
	for (int i = 0; i<joint_positions_begin.size(); i++)
	{
		displacement = joint_positions_begin[i].euclidean_distance_(joint_positions_end[i]);
		
		if( displacement > max_displacement)
		{
			max_displacement = displacement;
		}
	}
	
	// [3] to ensure no joint moves more than 0.03 m per sample, the number of 
	// samples needed = 2 + max_displacement/0.03 rounded down
	// (max_displacement/0.03 rounded down is the number of samples needed 
	// between begin and end, but you need another 2 for begin and end themselves)
	int samples_needed = static_cast<int>(max_displacement/0.03) + 2;
	
	// [4] sample linearly from begin to end with the number of samples found in step [3]
	VectorXf step_size = (end-begin)/(samples_needed-1);
	
	vector<VectorXf> configuration_samples (samples_needed);
	for (int i = 0; i<samples_needed; i++)
	{
		configuration_samples[i] = begin + i*step_size;
	}
	
	// [5] check the configuration of all samples. If none result in collision, 
	// the whole path is assumed to not cause collisions
	bool collision_found = false;
	int i = 0;
	while ((i<configuration_samples.size()) and !collision_found)
	{
		collision_found = check_collision_single_configuration(configuration_samples[i]);
		
		i++;
	}
	
	return collision_found;
}


// This function checks for a single configuration of the youbot, whether the 
// youbot is in collision when it is in that configuration.
// It first divides the youbot up into 8 spheres (whose positions depend on the 
// configuration). Then it uses those spheres to in order check whether the youbot
// is in collision with the floor, itself and the obstacles defined in the pointcloud.
// Returns true when a collision is found and false if the configuration is
// collision-free
bool check_collision_single_configuration(VectorXf configuration)
{
	vector<coordinate_3d> joint_positions = configuration_to_joint_positions(configuration);
	vector<coordinate_3d> youbot_spheres = divide_youbot_into_spheres(joint_positions);
	
	bool collision_found = check_collision_with_floor(youbot_spheres);
	
	if(!collision_found)
	{
		collision_found = check_self_collision(youbot_spheres);
		
		if(!collision_found)
		{
			collision_found = check_collision_with_pointcloud(youbot_spheres);
		}
	}
	return collision_found;
}


// This function checks whether any of the spheres that represent the youbot 
// (except for the one that represents the base) are in collision with any of 
// the points in the point cloud. The base is modelled as a cilinder instead of
// a sphere before being checked against the pointcloud.
// Returns true when a collision is found and false otherwise.
bool check_collision_with_pointcloud(vector<coordinate_3d> youbot_spheres)
{
	bool collision_found = false;
	
	
	// Make sure that all points that are out of the youbot's reach for its 
	// current base position are ignored
	vector<coordinate_3d> relevant_points = point_cloud_environment;
	int nr_of_relevant_points = 0;
	
	coordinate_3d sphere_of_relevance (youbot_spheres[0].x_, youbot_spheres[0].y_, 0.3, 0.7);
	
	for(int i = 0; i < point_cloud_environment.size(); i++)
	{
		if(sphere_of_relevance.in_collision_with_(point_cloud_environment[i]))
		{
			relevant_points[nr_of_relevant_points] = point_cloud_environment[i];
			nr_of_relevant_points++;
		}
	}
	
	relevant_points.erase(relevant_points.begin()+nr_of_relevant_points, relevant_points.end());
	
	
	// Check whether the base itself is in collision with the point cloud
	collision_found = check_collision_base_with_pointcloud(youbot_spheres[0], relevant_points);
	
	
	// Check whether any of the speheres that represent the arm segments are in
	// collision with the point cloud
	int i = 1; // sphere 0 represents the base, but that one will not be considered here as it will be represented as a cilinder instead of a sphere
	int j = 0;
	
	while((!collision_found) and (i<youbot_spheres.size()))
	{
		while((!collision_found) and (j<relevant_points.size()))
		{
			collision_found = youbot_spheres[i].in_collision_with_(relevant_points[j]);
			
			j++;
		}
		
		i++;
	}
	
	return collision_found;
}


// This function models the base as a cilinder with radius 0.4 m and height 0.15 m.
// It then checks whether any of the points in the pointcloud are in that cilinder.
// Returns true when a collision is found and false otherwise.
bool check_collision_base_with_pointcloud(coordinate_3d base, vector<coordinate_3d> relevant_points)
{
	bool collision_found = false;
	int i =0;
	double radial_distance;
	
	while( (i<relevant_points.size()) and !collision_found)
	{
		if (relevant_points[i].z_ < 0.15)
		{
			double radial_distance = sqrt( pow(base.x_-relevant_points[i].x_, 2) + pow(base.y_-relevant_points[i].y_, 2));
			
			if (radial_distance < 0.4)
			{
				collision_found = true;
			}
		}
		
		i++;
	}
	
	return collision_found;
}


// This function checks for all spheres that model the youbot (except the one 
// representing the base) whether they are colliding with the floor (z = 0).
// Returns true when a collision is found and false otherwise
bool check_collision_with_floor(vector<coordinate_3d> youbot_spheres)
{
	bool collision_found = false;

	int i = 1; // the base (sphere 0) will always be colliding with the floor (as it is driving on top of it), so that one should be ignored
	
	while( (i<youbot_spheres.size()) and !collision_found)
	{
		collision_found = (youbot_spheres[i].z_ < youbot_spheres[i].radius_);
		
		i++;
	}
	
	return collision_found;
}



// This Function checks whether any 2 non-consecutive sections of the youbot are
// in collision with eachother (by comparing the spheres that represent those sections)
// Returns true when a collision is found and false otherwise
bool check_self_collision(vector<coordinate_3d> youbot_spheres)
{
	bool collision_found = false;
		
	// Check base not in collision with any of the arm segments
	int i = 2;
	while( (i<youbot_spheres.size()) and !collision_found)
	{
		collision_found = youbot_spheres[0].in_collision_with_(youbot_spheres[i]);
		i++;
	}
	
	// check arm base in collision with the 2nd and 3rd arm segment
	i = 4;
	while( (i<youbot_spheres.size()) and !collision_found)
	{
		collision_found = youbot_spheres[1].in_collision_with_(youbot_spheres[i]);
		
		i++;
	}
	
	// check 1st arm segment (both upper and lower half) in collision with 3rd arm segment
	i = 6;
	while( (i<youbot_spheres.size()) and !collision_found)
	{
		collision_found = youbot_spheres[2].in_collision_with_(youbot_spheres[i]);
		
		if (!collision_found)
		{
			collision_found = youbot_spheres[3].in_collision_with_(youbot_spheres[i]);
		}
		
		i++;
	}
	
	return collision_found;
}


// This function calculates for a given configuration, where in the 3D workspace
// all the joints (and the base and arm_base) are.
// input configuration is {X, Y, theta, psi, alpha, beta, gamma} (order can be changed if needed)
// output coordinate vector is {center_base (height 0), center_arm_base, joint_1, joint_2, joint_3, endpoint}
vector<coordinate_3d> configuration_to_joint_positions(VectorXf configuration)
{
	double X = configuration[0]; // x-position base_center
	double Y = configuration[1]; // y-position base_center
	double theta = configuration[2]; // heading angle
	double psi = configuration[3]; // angle between the base and the shoulder of the arm
	double alpha = configuration[4]; // first joint angle
	double beta = configuration[5]; // second joint angle
	double gamma = configuration[6]; // third joint angle
	
	vector<coordinate_3d> joint_positions (6); // first entry is position of the 
	// center of the base (with z=0), after that the center point of the arm base, 
	// followed by the joints from 1 --> 3 and lastly the endpoint
	
	coordinate_3d base_center (X, Y, 0.0);
	joint_positions[0] = base_center;
	
	
	coordinate_3d arm_base_center (0.151*cos(theta) + X, 0.151*sin(theta) + Y, 213.5);
	joint_positions[1] = arm_base_center;
	
	
	double x_1 = 0.033*cos(theta + psi) + 0.151*cos(theta) + X;
	double y_1 = 0.033*sin(theta + psi) + 0.151*sin(theta) + Y;
	double z_1 = 0.287;
	coordinate_3d joint_1 (x_1, y_1, z_1);
	joint_positions[2] = joint_1;
	
	
	double x_2 = x_1 + 0.155*sin(alpha)*cos(theta + psi);
	double y_2 = y_1 + 0.155*sin(alpha)*sin(theta + psi);
	double z_2 = z_1 + 0.155*cos(alpha);
	coordinate_3d joint_2 (x_2, y_2, z_2);
	joint_positions[3] = joint_2;
	
	
	double x_3 = x_2 + 0.135*sin(alpha + beta)*cos(theta + psi);
	double y_3 = y_2 + 0.135*sin(alpha + beta)*sin(theta + psi);
	double z_3 = z_2 + 0.135*cos(alpha + beta);
	coordinate_3d joint_3 (x_3, y_3, z_3);
	joint_positions[4] = joint_3;
	
	
	double x_4 = x_3 + 0.218*sin(alpha + beta + gamma)*cos(theta + psi);
	double y_4 = y_3 + 0.218*sin(alpha + beta + gamma)*sin(theta + psi);
	double z_4 = z_3 + 0.218*cos(alpha + beta + gamma);
	coordinate_3d endpoint (x_4, y_4, z_4);
	joint_positions[5] = endpoint;
	
	return joint_positions;
}


// This function divides the youbot into 8 spheres, when the joint positions of the youbot are given
// Input (positions of): 
// {center_base (height 0), center_arm_base, joint_1, joint_2, joint_3, endpoint}
// Output (spheres with proper radius, representing): 
// {Base, Arm_base, lower_half_of_segment_1, upper_half_of_segment_1, lower_half_of_segment_2, upper_half_of_segment_2, lower_half_of_segment_3, upper_half_of_segment_3}
vector<coordinate_3d> divide_youbot_into_spheres(vector<coordinate_3d> joint_positions)
{
	vector<coordinate_3d> youbot_spheres (8);
	
	coordinate_3d base (joint_positions[0].x_, joint_positions[0].y_, -1.0, 1.2);
	youbot_spheres[0] = base;
	
	coordinate_3d arm_base (joint_positions[1].x_, joint_positions[1].y_, joint_positions[1].z_, 0.14);
	youbot_spheres[1] = arm_base;
	
	coordinate_3d translation_1_to_2 = joint_positions[2].vector_to_(joint_positions[3]);
	coordinate_3d translation_2_to_3 = joint_positions[3].vector_to_(joint_positions[4]);
	coordinate_3d translation_3_to_e = joint_positions[4].vector_to_(joint_positions[5]);
	
	coordinate_3d segment_1_lower  (joint_positions[2].x_+0.333*translation_1_to_2.x_, 
									joint_positions[2].y_+0.333*translation_1_to_2.y_, 
									joint_positions[2].z_+0.333*translation_1_to_2.z_,
									0.1);
	youbot_spheres[2] = segment_1_lower;
	
	
	coordinate_3d segment_1_upper  (joint_positions[2].x_+0.667*translation_1_to_2.x_, 
									joint_positions[2].y_+0.667*translation_1_to_2.y_, 
									joint_positions[2].z_+0.667*translation_1_to_2.z_,
									0.1);
	youbot_spheres[3] = segment_1_upper;
	
	
	
	coordinate_3d segment_2_lower  (joint_positions[3].x_+0.333*translation_2_to_3.x_, 
									joint_positions[3].y_+0.333*translation_2_to_3.y_, 
									joint_positions[3].z_+0.333*translation_2_to_3.z_,
									0.097);
	youbot_spheres[4] = segment_2_lower;
	
	
	coordinate_3d segment_2_upper  (joint_positions[3].x_+0.667*translation_2_to_3.x_, 
									joint_positions[3].y_+0.667*translation_2_to_3.y_, 
									joint_positions[3].z_+0.667*translation_2_to_3.z_,
									0.097);
	youbot_spheres[5] = segment_2_upper;
	
	
	
	coordinate_3d segment_3_lower  (joint_positions[4].x_+0.333*translation_3_to_e.x_, 
									joint_positions[4].y_+0.333*translation_3_to_e.y_, 
									joint_positions[4].z_+0.333*translation_3_to_e.z_,
									0.112);
	youbot_spheres[6] = segment_3_lower;
	
	
	coordinate_3d segment_3_upper  (joint_positions[4].x_+0.667*translation_3_to_e.x_, 
									joint_positions[4].y_+0.667*translation_3_to_e.y_, 
									joint_positions[4].z_+0.667*translation_3_to_e.z_,
									0.112);
	youbot_spheres[7] = segment_3_upper;
	
	return youbot_spheres;
}
