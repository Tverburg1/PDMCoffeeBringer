#ifndef three_D_coordinate_header
#define three_D_coordinate_header


// Class for defining the position and size of a point or sphere in 3D space
class coordinate_3d
{
	public:
		double x_; // x-coordinate
		double y_; // y-coordinate
		double z_; // z-coordinate
		double radius_; // radius of the sphere (in the case of a point, radius_ = 0)
		
		coordinate_3d(double x, double y, double z); // constructor for making a point
		coordinate_3d(); // constructor for making a default point in the origin
		coordinate_3d(double x, double y, double z, double radius); // constructor for making a sphere
		
		// Function to compute the euclidean distance between this point and the other given point
		double euclidean_distance_(coordinate_3d other_point);
		
		// Function for checking whether 2 spheres (or 1 sphere and 1 point) are in collision
		// Returns true when a collision is found and false otherwise.
		bool in_collision_with_(coordinate_3d other_point);
		
		// Function for cumputing the translation vector to go from this point to the other point.
		// the translation in x-direction is stored in output.x_, in y-direction in output.y_ 
		// and in z-direction in output.z_ (output.radius_ = 0.0, but has no physical meaning here)
		coordinate_3d vector_to_(coordinate_3d other_point);
};



#endif
