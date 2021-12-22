#include<fstream>
#include<vector>
#include<3d_coordinate.h>
#include"load_pointcloud.h"
#include<iostream>
#include<string>

using namespace std;

// Global variable for storing the pointcloud once it has been loaded
vector<coordinate_3d> point_cloud_environment;


// Function that loads the pointcloud stored as x y z coordinates in the file 
// whose filepath is provided. The loaded pointcloud is stored in the global 
// variable point_cloud_environment (see above)
// (returns 0 when everything is ok or 1 otherwise)
int load_pointcloud_data (string filepath) // currently, filepath should be "../pointcloud/scene3_filtered.xyz"
{
	cout << "Loading pointcloud" << endl;
	
	ifstream pointcloud_file;
	pointcloud_file.open(filepath);
	
	vector<coordinate_3d> loaded_points;

	while(pointcloud_file)
	{
		double x,y,z;
		
		for( int i=0; i<3; i++)
		{
			if(0 == i)
			{
				pointcloud_file >> x;
			}
			else if (1 == i)
			{
				pointcloud_file >> y;
			}
			else
			{
				pointcloud_file >> z;
			}
		}				
		
		if((z > 0.025) and (z < 0.8)) // ignore the floor and all points that are too high for the youbot to reach (as they are irrelevant for the collision detection)
		{
			coordinate_3d new_point (x, y, z);
			loaded_points.push_back(new_point);
		}
	}
	
	if(loaded_points.size() > 0)
	{
		cout << "Loading finished, pointcloud consists of " << loaded_points.size() << " points." << endl;
		point_cloud_environment = loaded_points;
		return 0;
	}
	else
	{
		cout << "Loading failed, either the file does not exist or the pointcloud has no points withing the youbot's reach" << endl;
		return 1;
	}
}

