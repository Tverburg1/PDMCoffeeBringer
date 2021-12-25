#include <iostream>
#include <fstream>
#include <cstdio>
#include <random>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
// include the files for collision
#include "collision_check.h"
#include "load_pointcloud.h"
#include "3d_coordinate.h"

using namespace std ;
using namespace Eigen;

const float EPS = 1e-6;

const int num_dim = 7;
const int max_iterations = 6000;

const int RADIUS = 5 ; // Radius of region around the goal point for which success is defined
const float GOAL_SAMPLING_PROB = 0.05;
const float INF = 1e18;

//Load the point cloud in vector<coordinate_3d> point_cloud_environment
int data_ok = load_pointcloud_data ("../collision_checks/point_cloud/scene3_filtered.xyz.txt");

// Function that obtains the maximum and minimum x and y values
vector<float> get_min_max_pc(vector<coordinate_3d> point_cloud){
    float xmax, ymax = -INF;
    float xmin, ymin = INF;

    for(int i=0;i<point_cloud_environment.size();i++) {
        float x = point_cloud_environment[i].x_;
        float y = point_cloud_environment[i].y_;

        if (xmax < x) { xmax = x; }
        if (xmin > x) { xmin = x; }
        if (ymax < y) { ymax = y; }
        if (ymin > y) { ymin = y; }
    }
    vector<float> bounds_2d{xmin,xmax,ymin,ymax};
    return bounds_2d;
}
vector<float> bounds_2d = get_min_max_pc(point_cloud_environment);
//vector<float> bounds_2d{0,800,0,600};

// The bounds of the dimensions of the configuration space should be defined here

MatrixXf get_bounds(){
    MatrixXf bounds(num_dim, 2) ;
    bounds(0,0) = 0;
//            bounds_2d[0], bounds_2d[1],
//             bounds_2d[2], bounds_2d[3] ,
//             0.0, 2 * M_PI ,
//             -169 / 180 * M_PI, 169 / 180 * M_PI ,
//             -65 / 180 * M_PI, 90 / 180 * M_PI ,
//             -146 / 180 * M_PI, 150 / 180 * M_PI ,
//             -102.5 / 180 * M_PI, 102.5 / 180 * M_PI


    return bounds;
}
MatrixXf bounds = get_bounds();

VectorXf diff = bounds.col(1)-bounds.col(2);
float JUMP_SIZE = diff.sum()/(pow(100.0,num_dim))/1.5; // same as before but now generalized
float DISK_SIZE = JUMP_SIZE ; // Ball radius around which nearby points are found

VectorXf start(num_dim), stop(num_dim) ;

vector < VectorXf > nodes(num_dim) ;
vector < int > parent, nearby ;
vector < float > cost, jumps ;
int nodeCnt = 0, goalIndex = -1 ;

bool pathFound = 0 ;

//extra variables for the ellipse and informed sampling
float cmin;
float half_width;
float half_height;
float cbest = INF;

//create matrices for designing the ellipsoid
MatrixXf M;
VectorXf midline(num_dim), center(num_dim);
MatrixXf C(num_dim, num_dim);
MatrixXf L;

vector< VectorXf > sampled_nodes(num_dim);

vector < VectorXf > get_path(){
    vector < VectorXf > optimal_path(num_dim);
    // loop that retraces the shortest path from the goal to start
    int node = goalIndex;
    optimal_path.push_back(nodes[node]);
    while (parent[node] != node) {
        int par = parent[node];
        optimal_path.push_back(nodes[par]);
        node = par;
    }
    // reverse the order of nodes to obtain list from start to stop
    reverse(optimal_path.begin(), optimal_path.end());
    return optimal_path;
}

void write_to_file(vector < VectorXf > optimal_path){
    ofstream my_file("optimal_path.txt");
    for(int i = 0; i<optimal_path.size();i++){
        my_file << optimal_path[i].transpose();
    }
    cout<< "Path written to file: optimal_path.txt"<<endl;
    my_file.close();
}


template <typename T> // Returns a random number in [low, high]
T randomCoordinate(T low, T high){
    random_device random_device;
    mt19937 engine{random_device()};
    uniform_real_distribution<float> dist(low, high);
    return dist(engine);
}

// Returns a random point with some bias towards goal
VectorXf pickRandomPoint() {
    VectorXf random_point(num_dim);
    float random_sample = randomCoordinate(0.0, 1.0);
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + VectorXf::Constant(num_dim, RADIUS) ;
    //fill in random point, might do this in a loop later on
    for(int i = 0;i<num_dim;i++){
        random_point(i) = randomCoordinate(bounds(i,0), bounds(i,1));
    }
    return random_point;
}

//Calculate the distance between two points
float distance(VectorXf a, VectorXf b) {
    VectorXf c = a-b;
    return sqrt(c.dot(c));
}

// Very simple steering function: If the distance between two points is too large, a point will be generated
// along that path max_dist away from the begin point.
VectorXf steering(VectorXf begin, VectorXf end, float max_dist) {
    float dist = distance(begin, end);
    if ((dist - max_dist)<=EPS){
        return end;
    }
    else {
        VectorXf delta = (end-begin)*max_dist/dist;
        return begin+delta;
    }
}

// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(VectorXf begin, VectorXf end) {
    return check_collision_between_two_configurations(begin,end);
}

// Returns true of an edge intersects the hypersphere with radius RADIUS around the stop point
// done by sampling n points on the edge
void checkDestinationReached() {
    VectorXf last_node = nodes.back();
    VectorXf par_last_node = nodes[parent[nodeCnt - 1]];
    VectorXf direction = last_node-par_last_node;

    int n = 10;
    for (int i =0; i<n; i++) {
        VectorXf sample = last_node - i * direction / n;
        if (distance(sample, stop) < RADIUS) { pathFound = 1; }
    }
    if(pathFound == 1) {
        goalIndex = nodeCnt - 1;
        cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl;
    }
}

/* Inserts nodes on the path from rootIndex till Point q such
   that successive nodes on the path are not more than
   JUMP_SIZE distance away */
void insertNodesInPath(int rootIndex, VectorXf& q) {
	VectorXf p = nodes[rootIndex] ;
	if(!isEdgeObstacleFree(p, q)) return ;
	while(!(p == q)) {
		VectorXf nxt = steering(p, q, JUMP_SIZE);
		nodes.push_back(nxt);
		parent.push_back(rootIndex);
		cost.push_back(cost[rootIndex] + distance(p, nxt));
		rootIndex = nodeCnt++ ;
		p = nxt ;
	}
}

/*  Rewires the parents of the tree greedily starting from
	the new node found in this iteration as the parent */
void rewire() {
	int lastInserted = nodeCnt - 1 ;
	for(auto nodeIndex: nearby) {
		int par = lastInserted, cur = nodeIndex;

		// Rewire parents as much as possible (greedily)
		while( ((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]) <= EPS) {
			int oldParent = parent[cur] ;
			parent[cur] = par; cost[cur] = cost[par] + distance(nodes[par], nodes[cur]);
			par = cur, cur = oldParent;
		}
	}
}



/* Compute parameters of the elliptical sampling region in 2D*/
void ellipse_param(float cbest){
    cmin = distance(start, stop);
    midline = (stop-start);
    center = start + midline/2;

    half_width = cbest/2;
    half_height = sqrt(cbest*cbest-cmin*cmin)/2;

    // Construct matrices
    VectorXf one = MatrixXf::Identity(num_dim,num_dim).col(0);

    //construct rotation matrix
    M = (midline.normalized())*one.transpose();
    JacobiSVD<MatrixXf> svd_of_M(M, ComputeFullU|ComputeFullV);

    MatrixXf U = svd_of_M.matrixU();
    MatrixXf V = svd_of_M.matrixV();

    MatrixXf diag = MatrixXf::Identity(num_dim,num_dim);
    diag(num_dim-1,num_dim-1) = U.determinant();
    diag(num_dim-2,num_dim-2) = V.determinant();

    C = U*diag*V.transpose();

    //Construct L-matrix
    L = MatrixXf::Identity(num_dim,num_dim)* half_height;
    L(0) = half_width;
}

VectorXf informed_sampling(){
    // compute the intervals, every thing above can be preemptively calculated
    VectorXf x_tf(num_dim);
    bool ok_point = false;
    while(!ok_point) {
        //Transform the random point via translation and rotation
        VectorXf x_random = VectorXf::Random(num_dim);
        x_tf = C*L*x_random + center;

        // check if within bounds for all
        int out = 0;
        for (int i = 0; i<num_dim; i++){
            if(x_tf(i)< bounds(i,0) or x_tf(i)>bounds(i,1)) {
                out++;
            }
        }
        if (out == 0){ok_point=true;}
    }
    return x_tf;
}


/*	Runs one iteration of RRT depending on user choice
	At least one new node is added on the screen each iteration. */
void RRT() {
	VectorXf newPoint(num_dim), nearestPoint(num_dim), nextPoint(num_dim) ; bool updated = false ; int cnt = 0 ;
	int nearestIndex = 0 ; float minCost = INF; nearby.clear(); jumps.resize(nodeCnt);

	while(!updated) {
        if (!pathFound) {
            newPoint = pickRandomPoint();
        }
        else {
            if (cost[goalIndex] < cbest) { // if a shorter path has been found recompute sampling region
                cbest = cost[goalIndex];
                ellipse_param(cbest);
            }
            // sample from the ellipse
            newPoint = informed_sampling();
        }


        // Find nearest point to the newPoint such that the next node
        // be added in graph in the (nearestPoint, newPoint) while being obstacle free
        nearestPoint = *nodes.begin();
        nearestIndex = 0;
        for (int i = 0; i < nodeCnt; i++) {
            if (pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while
                cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);

            // Make smaller jumps sometimes to facilitate passing through narrow passages
            jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE;
            auto pnt = nodes[i];
            if ((distance(pnt, newPoint) - distance(nearestPoint, newPoint)) <= EPS and
                isEdgeObstacleFree(pnt, steering(pnt, newPoint, jumps[i])))
                nearestPoint = pnt, nearestIndex = i;
        }
        nextPoint = steering(nearestPoint, newPoint, jumps[nearestIndex]);
        if (!isEdgeObstacleFree(nearestPoint, nextPoint)) continue;

        if (!pathFound) {
            // This is where we don't do any RRT* optimization part
            updated = true;
            nodes.push_back(nextPoint);
            nodeCnt++;
            parent.push_back(nearestIndex);
            cost.push_back(cost[nearestIndex] + distance(nearestPoint, nextPoint));
            if (!pathFound) checkDestinationReached();
            continue;
        }

        // Find nearby nodes to the new node within radius optimal radius opt_r (function of DISK SIZE) of the new node
        for (int i = 0; i < nodeCnt; i++) {
            float opt_r = DISK_SIZE * pow((log(nodeCnt) / nodeCnt), 1 / (num_dim + 1));
            if ((distance(nodes[i], nextPoint) - opt_r) <= EPS and isEdgeObstacleFree(nodes[i], nextPoint))
                nearby.push_back(i);
        }

		// Find minimum cost path to the new node
		int par = nearestIndex; minCost = cost[par] + distance(nodes[par], nextPoint);
		for(auto nodeIndex: nearby) {
			if( ( (cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint)) - minCost) <= EPS)
				minCost = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint), par = nodeIndex;
		}

		parent.push_back(par); cost.push_back(minCost);
		nodes.push_back(nextPoint); nodeCnt++; sampled_nodes.push_back(nextPoint);
		updated = true ;
		if(!pathFound) checkDestinationReached();
		rewire();
	}
}

int main() {
//    int data_ok = load_pointcloud_data ("./scene3_filtered.xyz");
    int data_ok = 0;

    if (data_ok == 1){return 1;}

    else {
        nodeCnt = 1;
        nodes.push_back(start);
        int iterations = 0;
        parent.push_back(0);
        cost.push_back(0);

        while (iterations < max_iterations) {
            RRT();
            iterations++;

            if (iterations % 500 == 0) {
                cout << "Iterations: " << iterations << endl;
                if (!pathFound) cout << "Not reached yet :( " << endl;
                else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl;
                cout << endl;
            }

        }
        cout << "Number of iterations: " << iterations << endl;
        if (!pathFound) cout << "Path not reached yet :( " << endl;
        else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl;

        vector<VectorXf> optimal_path = get_path();
        write_to_file(optimal_path);
    }

}


