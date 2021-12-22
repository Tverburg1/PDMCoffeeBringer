#include"3d_coordinate.h"
#include<cmath>
#include<math.h>

using namespace std;

// constructor for making a point
coordinate_3d::coordinate_3d(double x, double y, double z)
{
	x_ = x;
	y_ = y;
	z_ = z;
	radius_ = 0.0;
}


// constructor for making a sphere
coordinate_3d::coordinate_3d(double x, double y, double z, double radius)
{
	x_ = x;
	y_ = y;
	z_ = z;
	radius_ = radius;
}


// constructor for making a default point in the origin
coordinate_3d::coordinate_3d()
{
	x_ = 0.0;
	y_ = 0.0;
	z_ = 0.0;
	radius_ = 0.0;
}


// function for computing the eucliden distance between this point and another given point
double coordinate_3d::euclidean_distance_(coordinate_3d other_point)
{
	double euclidean_distance = sqrt( pow(x_-other_point.x_, 2) + pow(y_-other_point.y_, 2) + pow(z_-other_point.z_, 2));
	return euclidean_distance;
}


// Function for checking whether 2 spheres (or 1 sphere and 1 point) are in collision
// Returns true when a collision is found and false otherwise.
bool coordinate_3d::in_collision_with_(coordinate_3d other_point)
{
	double center_distance = euclidean_distance_(other_point);
	
	if(center_distance < (radius_ + other_point.radius_))
	{
		return true;
	}
	else
	{
		return false;
	}
}


// Function for cumputing the translation vector to go from this point to the other point.
// the translation in x-direction is stored in output.x_, in y-direction in output.y_ 
// and in z-direction in output.z_ (output.radius_ = 0.0, but has no physical meaning here)
coordinate_3d coordinate_3d::vector_to_(coordinate_3d other_point)
{
	coordinate_3d translation_vector(other_point.x_-x_, other_point.y_-y_, other_point.z_-z_);
	return translation_vector;
}






















