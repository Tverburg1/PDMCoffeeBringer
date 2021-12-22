#ifndef pointcloud_load_header
#define pointcloud_load_header

#include<vector>
#include<3d_coordinate.h>
#include<string>

// Global variable for storing the pointcloud once it has been loaded
extern std::vector<coordinate_3d> point_cloud_environment;

// Function that loads the pointcloud stored as x y z coordinates in the file 
// whose filepath is provided. The loaded pointcloud is stored in the global 
// variable point_cloud_environment (see above)
// (returns 0 when everything is ok or 1 otherwise)
int load_pointcloud_data (std::string filepath);

#endif
