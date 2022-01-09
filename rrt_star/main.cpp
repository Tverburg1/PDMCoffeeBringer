#include <iostream>
#include <fstream>
#include <cstdio>
#include <random>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <SFML/Graphics.hpp>
// include the files for collision
#include "collision_check.h"
#include "load_pointcloud.h"
#include "3d_coordinate.h"

using namespace std ;
using namespace Eigen;


const float EPS = 1e-6;

const int num_dim = 7;
int max_iterations = 10000;


float RADIUS = 1.0 ; // Radius of region around the goal point for which success is defined
const float GOAL_SAMPLING_PROB = 0.10;
const float INF = 1e18;
using Vector7f = Matrix<float, 7, 1> ;

float JUMP_SIZE = 0.5;
float DISK_SIZE = JUMP_SIZE ; // Ball radius around which nearby points are found

int scaling_factor = 30; //scaling for plotting window

Vector7f start, stop ;
// Get input
template <typename T>
void redefine(string title, T &variable){
    float answer=0;
    cout << title<< ":" <<endl;
    cin>> answer;
    if (answer){variable=answer;}
    else {cout<<"default used: "<< variable <<endl;}
}

void getInput() {
    float answer;
    cout << "Select here parameters for running the code. "
            "If you input 0 default values will be used" << endl<<endl; ;

    cout << "redefine start? (1 for yes, 0 for no)"<<endl;
    cin>> answer;
    if (answer){
        cout << "Input start configuration (input 7 numbers separated by spaces)" <<endl;
        cin >> start(0)>>start(1)>>start(2)>>start(3)>>start(4)>>start(5)>>start(6);
    }

    cout << "redefine stop? (1 for yes, 0 for no)"<<endl;
    cin>> answer;
    if (answer){
        cout << "Input stop configuration (input 7 numbers separated by spaces)" <<endl;
        cin >> stop(0)>>stop(1)>>stop(2)>>stop(3)>>stop(4)>>stop(5)>>stop(6);
    }

    cout << "redefine other parameters? (1 for yes, 0 for no)"<<endl;
    cin>> answer;
    if (answer){
        redefine("Goal Radius", RADIUS);
        redefine("Maximum jump distance", JUMP_SIZE);
        redefine("Radius around which nearby points are found", DISK_SIZE);
        redefine("Maximum iterations", max_iterations);
    }
}

// Function for making a top-down projection of the point cloud for visualisation
vector<Vector2f> project_2d(vector<coordinate_3d> point_cloud){
    vector<Vector2f> points;
    for(int i=0;i<point_cloud.size();i++) {
        if (point_cloud[i].z_ < 0.5*1.1 and point_cloud[i].z_>0.5*0.9){
            float x = point_cloud[i].x_;
            float y = point_cloud[i].y_;
            Vector2f point; point << x, y;
            points.push_back(point);
        }
    }
    return points;
}
vector<Vector2f> proj_pc_points;

// Function that obtains the maximum and minimum x and y values from point cloud
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
//vector<float> bounds_2d = get_min_max_pc(point_cloud_environment);
//vector<float> bounds_2d{-10.4, 6.13,-4.68,18.31};
vector<float> bounds_2d;

// The bounds of the dimensions of the configuration space should be defined here

Matrix<float, num_dim, 2> get_bounds(){
    Matrix<float, num_dim, 2> bounds;
    bounds <<
        bounds_2d[0], bounds_2d[1],
        bounds_2d[2], bounds_2d[3] ,
        0.0, (2 * M_PI) ,
        (-169.0/180*M_PI), (169.0/180*M_PI ),
        (-65.0 / 180 * M_PI), (90.0 / 180 * M_PI) ,
        (-146.0 / 180 * M_PI), (150.0 / 180 * M_PI) ,
        (-102.5 / 180 * M_PI), (102.5 / 180 * M_PI);

    return bounds;
}
Matrix<float, num_dim, 2> bounds;

//Vector7f diff = bounds.col(1)-bounds.col(0);
//float JUMP_SIZE = diff.sum()/(pow(100.0,num_dim)); // same as before but now generalized


vector < Vector7f > nodes;
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
Vector7f midline, center;
MatrixXf C(num_dim, num_dim);
MatrixXf L;

vector< Vector7f > sampled_nodes;

vector < Vector7f > get_path(){
    vector < Vector7f > optimal_path;
    // loop that retraces the shortest path from the goal to start
    int node = goalIndex;
    optimal_path.push_back(stop);
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

// width and height of plotting windows
int WIDTH;
int HEIGHT;

sf::Vector2f CV(VectorXf configuration){
    sf::Vector2f output;

    output.x = (configuration(0)-bounds_2d[0])*scaling_factor;
    output.y = HEIGHT-(configuration(1)-bounds_2d[2])*scaling_factor; //this is so that the y-axis points up
    return output;
}

void draw(sf::RenderWindow& window) {
    sf::Vertex line[2]; sf::CircleShape nodeCircle; sf::CircleShape pc_circle;

    // Uncomment if circular nodes are to be drawn
    for(auto& node: nodes) {
        nodeCircle.setRadius(RADIUS/2.5); // nodeCircle.setRadius(min(2.0, RADIUS/2.0));
        nodeCircle.setOrigin(RADIUS/2.5, RADIUS/2.5);
        nodeCircle.setFillColor(sf::Color(0, 255, 171)); nodeCircle.setPosition(CV(node));
        window.draw(nodeCircle);
    }

    // Draw obstacles
    for(auto& point: proj_pc_points){
        pc_circle.setRadius(RADIUS/2.5); // nodeCircle.setRadius(min(2.0, RADIUS/2.0));
        pc_circle.setOrigin(RADIUS/2.5, RADIUS/2.5);
        pc_circle.setFillColor(sf::Color(125,125,125)); pc_circle.setPosition(CV(point));
        window.draw(pc_circle);
    }
    // Draw edges between nodes
    for(int i = (int)nodes.size() - 1; i; i--) {
        Vector7f par = nodes[parent[i]] ;
        line[0] = sf::Vertex(sf::Vector2f(CV(par)));
        line[1] = sf::Vertex(sf::Vector2f(CV(nodes[i])));
        window.draw(line, 2, sf::Lines);
    }
    // draw start point and stop point
    sf::CircleShape startingPoint, endingPoint ;
    startingPoint.setRadius(RADIUS); endingPoint.setRadius(RADIUS);
    startingPoint.setFillColor(sf::Color::Green); endingPoint.setFillColor(sf::Color::Magenta);
    startingPoint.setPosition(CV(start)); endingPoint.setPosition(CV(stop));
    startingPoint.setOrigin(RADIUS/2, RADIUS/2); endingPoint.setOrigin(RADIUS/2, RADIUS/2);

    window.draw(startingPoint); window.draw(endingPoint);

    // If destination is reached then path is retraced and drawn
    if(pathFound) {
        vector < Vector7f > path = get_path();
        for(int i = 1; i< path.size(); i++){
            line[0] = sf::Vertex(CV(path[i-1]));
            line[1] = sf::Vertex(CV(path[i]));
            line[0].color = line[1].color = sf::Color::Red; // orange color
            window.draw(line, 2, sf::Lines);
        }

//        int node = goalIndex;
//        while(parent[node] != node) {
//            int par = parent[node];
//            line[0] = sf::Vertex(CV(nodes[par]));
//            line[1] = sf::Vertex(CV(nodes[node]));
//            line[0].color = line[1].color = sf::Color::Red; // orange color
//            window.draw(line, 2, sf::Lines);
//            node = par ;
//        }

        //draw ellipse
        sf::CircleShape ellipse;
        ellipse.setRadius(1);
        ellipse.setOrigin(1,1);
        ellipse.setScale(half_width*scaling_factor, half_height*0.5*scaling_factor);
        ellipse.setOutlineColor(sf::Color::Yellow);
        ellipse.setOutlineThickness(0.1*cmin);
        ellipse.setFillColor(sf::Color::Transparent);


        double angle = (atan2(midline(0),midline(1))- M_PI/2 )/ M_PI*180;
        ellipse.move(CV(center));
        ellipse.rotate(angle);

        window.draw(ellipse);
    }
}

void write_to_file(vector < Vector7f > optimal_path){
    ofstream my_file("../../optimal_path.txt");
    for(int i = 0; i<optimal_path.size();i++){
        my_file << optimal_path[i].transpose()<<endl;
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
// returns a vector of indices of the given array in descending order
//source: https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

    // initialize original index locations
    vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    stable_sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    return idx;
}

// Returns a random point with some bias towards goal
Vector7f pickRandomPoint() {
    Vector7f random_point;
    float random_sample = randomCoordinate(0.0, 1.0);
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound){
        for(int i = 0;i<num_dim;i++){
            random_point(i) = randomCoordinate(-RADIUS, RADIUS);
        }
        return stop + random_point ;
    }
    //fill in random point
    else{
        for(int i = 0;i<num_dim;i++){
            random_point(i) = randomCoordinate(bounds(i,0), bounds(i,1));
        }
        return random_point;
    }
}

//Calculate the distance between two points
float distance(Vector7f a, Vector7f b) {
    Vector7f c = a-b;
    return c.norm();
}

// Very simple steering function: If the distance between two points is too large, a point will be generated
// along that path max_dist away from the begin point.
Vector7f steering(Vector7f begin, Vector7f end, float max_dist) {
    float dist = distance(begin, end);
    if (dist <= max_dist){
        return end;
    }
    else {
        Vector7f delta = (end-begin)*max_dist/dist;
        return begin+delta;
    }
}

// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(Vector7f begin, Vector7f end) {
    return !check_collision_between_two_configurations(begin,end);
}

// Returns true of an edge intersects the hypersphere with radius RADIUS around the stop point
// done by sampling n points on the edge
void checkDestinationReached() {
    Vector7f last_node = nodes.back();
    Vector7f par_last_node = nodes[parent[nodeCnt - 1]];
    Vector7f direction = last_node-par_last_node;

    int n = 10;
    for (int i =0; i<n; i++) {
        Vector7f sample = last_node - i * direction / n;
        if (distance(sample, stop) < RADIUS) { pathFound = 1; }
    }
    if(pathFound == 1) {
        goalIndex = nodeCnt - 1;
        cout << "Reached!! With a distance of " << cost[goalIndex]+ distance(nodes[goalIndex],stop)<< " units. " << endl << endl;
    }
}

/* Inserts nodes on the path from rootIndex till Point q such
   that successive nodes on the path are not more than
   JUMP_SIZE distance away */
//void insertNodesInPath(int rootIndex, Vector7f& q) {
//	Vector7f p = nodes[rootIndex] ;
//	if(!isEdgeObstacleFree(p, q)) return ;
//	while(!(p == q)) {
//		Vector7f nxt = steering(p, q, JUMP_SIZE);
//		nodes.push_back(nxt);
//		parent.push_back(rootIndex);
//		cost.push_back(cost[rootIndex] + distance(p, nxt));
//		rootIndex = nodeCnt++ ;
//		p = nxt ;
//	}
//}

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

Vector7f informed_sampling(){
    // compute the intervals, every thing above can be preemptively calculated
    Vector7f x_tf;
    bool ok_point = false;
    while(!ok_point) {
        //Transform the random point via translation and rotation
        Vector7f x_random = Vector7f::Random();
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
	Vector7f newPoint, nearestPoint, nextPoint ; bool updated = false ;
	int nearestIndex = 0 ; float minCost = INF; nearby.clear(); jumps.resize(nodeCnt);

	while(!updated) {
        if (!pathFound) {
            newPoint = pickRandomPoint();
        }
        else {
            if (cost[goalIndex] + distance(nodes[goalIndex],stop) < cbest) { // if a shorter path has been found recompute sampling region
                cbest = cost[goalIndex] + distance(nodes[goalIndex],stop);
                ellipse_param(cbest);
            }
            // sample from the ellipse
            newPoint = informed_sampling();
        }


        // Find nearest point to the newPoint such that the next node
        // be added in graph in the (nearestPoint, newPoint) while being obstacle free
        nearestPoint = *nodes.begin();
        nearestIndex = 0;

        vector<float> distances(nodeCnt); // vector for storing the distances between points
        for (int i = 0; i < nodeCnt; i++) {
            if (pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while
                cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);

            // Make smaller jumps sometimes to facilitate passing through narrow passages
            jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE;

            //save distances between newPoint and each node
            distances[i] = distance(newPoint, nodes[i]);
        }
        // take the sorted distances and check for the closest node if a collision free edge can be created
        // if not go on to the next, if so break the loop and continue on
        for (auto i: sort_indexes(distances)) {
            Vector7f pnt = nodes[i];
            if (isEdgeObstacleFree(pnt, steering(pnt, newPoint, jumps[i]))) {
                nearestPoint = pnt, nearestIndex = i;
                break;
            }
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

        // This section should only be reachable when the path IS found. This is RRT*
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

		rewire();
	}
}

int main() {
    //Load the point cloud in vector<coordinate_3d> point_cloud_environment
    int data_ok = load_pointcloud_data("../../collision_checks/pointcloud/pc_edit.xyz");
    if (data_ok == 1) { return 1; }

    else {
        // some default values for start and stop, can be changed further on
        start = Vector7f::Constant(0);

        stop = Vector7f::Constant(0);
        stop(0) = 3;
        stop(1) = 10;

        getInput();

        bounds_2d = get_min_max_pc(point_cloud_environment);
        bounds = get_bounds();
        proj_pc_points = project_2d(point_cloud_environment);
        WIDTH = (bounds_2d[1]-bounds_2d[0]+2)*scaling_factor;
        HEIGHT = (bounds_2d[3]-bounds_2d[2]+2)*scaling_factor;
        sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Informed RRT_star");

        nodeCnt = 1;
        nodes.push_back(start);
        int iterations = 0;
        parent.push_back(0);
        cost.push_back(0);

        cout << endl << "Starting node is in Green and Destination node is in Magenta" << endl << endl;
        while (window.isOpen() and iterations < max_iterations) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                }
            }
            RRT();
            iterations++;

            if (iterations % 10 == 0) {
                cout << "Iterations: " << iterations << endl;
                if (!pathFound) cout << "Not reached yet :( " << endl;
                else cout << "Shortest distance till now: " << cbest<< " units." << endl;
                cout << endl;
            }

            //sf::sleep(delayTime);
            window.clear();
            draw(window);
            window.display();
        }

        cout << "Number of iterations: " << iterations << endl;
        if (!pathFound) cout << "Goal not reached within max iterations or manual termination " << endl;
        else {
            cout << "Shortest distance till now: " << cbest << " units." << endl;
            vector<Vector7f> optimal_path = get_path();
            write_to_file(optimal_path);
        }

    }
}
//
//        while (iterations < max_iterations and !pathFound) { // Is purely for testing, for actual use you should not stop once a path has been found
//            RRT();
//            iterations++;
//
//            if (iterations % 10 == 0) {
//                cout << "Number of iterations: " << iterations << endl;
//                if (!pathFound) cout << "Not reached yet" << endl;
//                else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl;
//                cout << endl;
//            }
//
//        }
//        cout << "Number of iterations: " << iterations << endl;
//        if (!pathFound) cout << "Goal not reached within max iterations :( " << endl;
//        else {
//            cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl;
//            vector<Vector7f> optimal_path = get_path();
//            write_to_file(optimal_path);
//        }
//    }
//
//}

