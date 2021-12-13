#include <iostream>
#include <cstdio>
#include <random>
#include <SFML/Graphics.hpp> // might have to change visualization method
#include "geometry.h" // If the obstacle collision is properly implemented, this can be removed
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

using namespace std ;
using namespace Eigen;

const double EPS = 1e-6;

const int num_dim = 2;

// The bounds of the dimensions of the configuration space should be defined here
const double WIDTH = 800 ;
const double HEIGHT = 600 ;
MatrixXf bounds(num_dim, 2) << 0, WIDTH, 0, HEIGHT;

const int RADIUS = 5 ; 
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

VectorXf diff = bounds.col(1)-bounds.col(2);
double JUMP_SIZE = diff.sum()/(pow(100.0,num_dim))/1.5; // same as before but now generalized
double DISK_SIZE = JUMP_SIZE ; // Ball radius around which nearby points are found

vector < Polygon > obstacles ; 
VectorXf start(num_dim), stop(num_dim) ;
int obstacle_cnt = 1 ;

vector < VectorXf > nodes(num_dim) ;
vector < int > parent, nearby ; 
vector < double > cost, jumps ; 
int nodeCnt = 0, goalIndex = -1 ; 

vector <sf::ConvexShape> polygons ;
sf::CircleShape startingPoint, endingPoint ; 
bool pathFound = 0 ;

//extra variables for the ellipse and informed sampling
double cmin;
double half_width;
double half_height;
double cbest = INF;

//create matrices for designing the ellipsoid
MatrixXf M;
VectorXf midline(num_dim), center(num_dim);
MatrixXf C(num_dim, num_dim);
MatrixXf L;

vector< VectorXf > sampled_nodes(num_dim);

// has to be replaced
void getInput() {
	cout << "NOTE:" << endl ; 
	cout << "Height of screen: " << HEIGHT << " pixels." ;
	cout << " Width of screeen: " << WIDTH << " pixels." << endl ;
	cout << "Maximum distance by which algorithm jumps from one point to another: " << JUMP_SIZE << " units" << endl ;
	cout << "If you would like to change of any of these, please make modifications in code" << endl ; 
	cout << "Please provide your inputs keeping this in mind. " << endl << endl ;

	cout << "Input co-ordinates of starting and ending point respectively in this format X1 Y1 X2 Y2" << endl ;
	cin >> start(0) >> start(1) >> stop(0) >> stop(1) ;
	cout << "How many obstacles?" << endl ; 
	cin >> obstacle_cnt ; 
	
	obstacles.resize(obstacle_cnt); 
	int pnts = 0 ; Point pnt ;
	vector < Point > poly ;
	
	for(int i = 0; i < obstacle_cnt; i++) {
		poly.clear();
		cout << "How many points in " << i+1 << "th polygon?" << endl ; 
		cin >> pnts ; 
		poly.resize(pnts);

		cout << "Input co-ordinates of " << i+1 << "th polygon in clockwise order" << endl ;
		for(int j = 0; j < pnts; j++) {
			cin >> pnt.x >> pnt.y ; 
			obstacles[i].addPoint(pnt);
		}
	}
}

// has to be replaced
// Prepares SFML objects of starting, ending point and obstacles 
void prepareInput() {
	// Make starting and ending point circles ready 
	startingPoint.setRadius(RADIUS); endingPoint.setRadius(RADIUS); 
    startingPoint.setFillColor(sf::Color(208, 0, 240)); endingPoint.setFillColor(sf::Color::Blue);
    startingPoint.setPosition(start(0), start(1)); endingPoint.setPosition(stop(0), stop(1));
    startingPoint.setOrigin(RADIUS/2, RADIUS/2); endingPoint.setOrigin(RADIUS/2, RADIUS/2);

    // Prepare polygon of obstacles 
	polygons.resize(obstacle_cnt);
	for(int i = 0; i < obstacle_cnt; i++) {
		polygons[i].setPointCount(obstacles[i].pointCnt); 
		polygons[i].setFillColor(sf::Color(89, 87, 98)); 
		for(int j = 0; j < obstacles[i].pointCnt; j++) 
			polygons[i].setPoint(j, sf::Vector2f(obstacles[i].points[j].x, obstacles[i].points[j].y));
	}
}

// might get removed or replaced
void draw(sf::RenderWindow& window) {
    if (num_dim == 2) {
        sf::Vertex line[2];
        sf::CircleShape nodeCircle;

        // Uncomment if circular nodes are to be drawn
        for (auto &node: sampled_nodes) {
            nodeCircle.setRadius(RADIUS / 2.5); // nodeCircle.setRadius(min(2.0, RADIUS/2.0));
            nodeCircle.setOrigin(RADIUS / 2.5, RADIUS / 2.5);
            nodeCircle.setFillColor(sf::Color(0, 255, 171));
            nodeCircle.setPosition(node(0), node(1));
            window.draw(nodeCircle);
        }

        // Draw obstacles
        for (auto &poly: polygons) window.draw(poly);

        // Draw edges between nodes
        for (int i = (int) nodes.size() - 1; i; i--) {
            VectorXf par = nodes[parent[i]];
            line[0] = sf::Vertex(sf::Vector2f(par(0), par(1)));
            line[1] = sf::Vertex(sf::Vector2f(nodes[i](0), nodes[i](1)));
            window.draw(line, 2, sf::Lines);
        }

        window.draw(startingPoint);
        window.draw(endingPoint);

        // If destination is reached then path is retraced and drawn
        if (pathFound) {
            int node = goalIndex;
            while (parent[node] != node) {
                int par = parent[node];
                line[0] = sf::Vertex(sf::Vector2f(nodes[par](0), nodes[par](1)));
                line[1] = sf::Vertex(sf::Vector2f(nodes[node](0), nodes[node](1)));
                line[0].color = line[1].color = sf::Color::Red; // orange color
                window.draw(line, 2, sf::Lines);
                node = par;
            }

            //draw ellipse
            if (use_informed_sampling) {
                sf::CircleShape ellipse;
                ellipse.setRadius(1);
                ellipse.setOrigin(1, 1);
                ellipse.setScale(half_width, half_height);
                ellipse.setOutlineColor(sf::Color::Yellow);
                ellipse.setOutlineThickness(0.008);
                ellipse.setFillColor(sf::Color::Transparent);
                double angle = acos(C(0, 0)) / M_PI * 180;
                ellipse.move(center(0), center(1));
                ellipse.rotate(angle);

                window.draw(ellipse);

                sf::CircleShape nd;
                nd.setRadius(RADIUS / 2.5); // nodeCircle.setRadius(min(2.0, RADIUS/2.0));
                nd.setOrigin(RADIUS / 2.5, RADIUS / 2.5);
                nd.setFillColor(sf::Color::Magenta);
                nd.setPosition(center(0), center(1));
                window.draw(nd);

                sf::Vertex mid_line[2];
                mid_line[0] = sf::Vertex(sf::Vector2f(start(0), start(1)));
                mid_line[1] = sf::Vertex(sf::Vector2f(stop(0), stop(1)));
                mid_line[0].color = mid_line[1].color = sf::Color::Green; // orange color
                window.draw(midline, 2, sf::Lines);
            }
        }
    }
}

template <typename T> // Returns a random number in [low, high] 
T randomCoordinate(T low, T high){
    random_device random_device;
    mt19937 engine{random_device()};
    uniform_real_distribution<double> dist(low, high);
    return dist(engine);
}

// Returns a random point with some bias towards goal
VectorXf pickRandomPoint() {
    VectorXf random_point(num_dim);
    double random_sample = randomCoordinate(0.0, 1.0);
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + VectorXf::Constant(num_dim, RADIUS) ;
    //fill in random point, might do this in a loop later on
    for(int i = 0;i<num_dim;i++){
        random_point(i) = randomCoordinate(bounds(i,0), bounds(i,1);
    }
    return random_point;
}

//Calculate the distance between two points
VectorXf distance(VectorXf a, VectorXf b) {
    VectorXf c = a-b;
    return sqrt(c.dot(c));
}

// Very simple steering function: If the distance between two points is too large, a point will be generated
// along that path max_dist away from the begin point.
VectorXf steering(VectorXf begin, vectorXf end, double max_dist) {
    double dist = distance(begin, end);
    if ((dist - max_dist)<=EPS){
        return end;
    }
    else {
        VectorXf delta = (end-begin)*max_dist/dist;
        return begin+delta;
    }
}

// has to be replaced
// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(Point a, Point b) {
    for(auto& poly: obstacles)
        if(lineSegmentIntersectsPolygon(a, b, poly))
        	return false ; 
    return true ; 
}

void checkDestinationReached() {
	if(checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), Point(stop(0), stop(1)), RADIUS)) {
		pathFound = 1 ; 
		goalIndex = nodeCnt - 1;
		cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl ; 
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
void ellipse_param(double cbest){
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

        // check if within bounds for all dimensions
        for (int i = 0; i<num_dim; i++){
            if(x_tf(i)< bounds(i,0) or x_tf(i)>bounds(i,1)) {
                ok_point = false;
            }
        }
        else{ok_point=true;}
    }
    return x_tf;
}


/*	Runs one iteration of RRT depending on user choice 
	At least one new node is added on the screen each iteration. */
void RRT() {
	VectorXf newPoint(num_dim), nearestPoint(num_dim), nextPoint(num_dim) ; bool updated = false ; int cnt = 0 ;
	int nearestIndex = 0 ; double minCost = INF; nearby.clear(); jumps.resize(nodeCnt);

	while(!updated) {
        if (!pathFound) {
            newPoint = pickRandomPoint();
        } else {
            if (cost[goalIndex] < cbest) {
                cbest = cost[goalIndex];
                ellipse_param(cbest);
            }
            // sample from the ellipse
            newPoint = informed_sampling();
        }

        else{
            newPoint = pickRandomPoint();
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
            double opt_r = DISK_SIZE * pow((log(nodeCnt) / nodeCnt), 1 / (num_dim + 1));
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
	getInput(); prepareInput();
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Basic Anytime RRT");

	nodeCnt = 1; nodes.push_back(start); int iterations = 0 ; 
	parent.push_back(0); cost.push_back(0);
    sf::Time delayTime = sf::milliseconds(5);

    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ; 
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
            	window.close();
            	return 0; exit(0);
            }
        }
        RRT(); iterations++;
        
		if(iterations % 500 == 0) {
			cout << "Iterations: " << iterations << endl ; 
			if(!pathFound) cout << "Not reached yet :( " << endl ;
			else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl ;
			cout << endl ;
		}

		//sf::sleep(delayTime);
		window.clear();
		draw(window); 
        window.display();
    }
}

/* SOME SAMPLE INPUTS ARE SHOWN BELOW (only cin part) without any RRT preference */ 

/*
100 70
600 400
2
4
200 480
200 100
250 100
250 480
5
400 0
300 100
350 250
450 250
500 100
*/

/*
50 50
750 180
3
4
100 0
200 0
200 400
100 400
4
250 200
350 200
350 600
250 600
4
400 50
550 50
550 250
400 250
*/

/*
50 50
750 580
3
4
100 0
200 0
200 400
100 400
4
250 200
350 200
350 600
250 600
4
400 32
700 32
700 575
400 575
*/

/*
190 30
660 500
5
3
200 50
200 200
350 200
3
220 50
370 50
370 200
3
400 250
400 450
600 450
3
430 250
630 250
630 450
3
640 470
640 550
680 550
*/

/*
190 55
660 500
9
4
740 360 
750 350
680 540
670 530
3
710 290
780 350
630 380
3
520 450
620 540
349 580
4
450 70
700 70
700 240 
450 240
3
200 50
200 200
350 200
3
220 50
370 50
370 200
3
400 250
400 450
600 450
3
430 250
630 250
630 450
3
640 470
640 550
680 550
*/