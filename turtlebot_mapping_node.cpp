//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen>

using namespace Eigen;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

//IPS Data
double ips_x;
double ips_y;
double ips_yaw;

//Laser Data
float angle_min;        // start angle of the scan [rad]
float angle_max;        // end angle of the scan [rad]
float angle_increment;  // angular distance between measurements [rad]
float scan_time; //time between scans (we can check for this to modify the rate of updates)
float range_min;
float range_max;

//laser rangedata:
std::vector <float> r;

//laser constants
float alpha = 1; 
float beta = 0.05; 

float time_elapsed = 0;   

//Map Properties
static const int RES_X = 100; //100m x direction map
static const int RES_Y = 100; //100m y direction map
static const int cell_size_in_m = 0.1; 
const float cell_not_occ = 0.4;
const float cell_occ = 0.6;

static const int M = RES_X/cell_size_in_m;
static const int N = RES_Y/cell_size_in_m;

Eigen::MatrixXd m(M,N) = 0.5*Eigen::MatrixXd::Ones(M,N);
Eigen::MatrixXd L0;
L0 = m.array() / (1-Qm.array());
L0 = L0.array().exp()  

Eigen::MatrixXd L;
L=L0;

//store position of robot
std::vector <double> robot_pose_x;
std::vector <double> robot_pose_y;
std::vector <double> robot_pose_yaw;



//bresenham vectors, cleared after init_final_occupancy_grid function

/*void polar_to_cartesian(float range, float &x, float &y, angle_increment, 
    int step, float angle_min, float angle_max)
{

    // sanity check
    if (angle_min + angle_increment*step > angle_max)
    {
         throw std::invalid_argument("invalid Theta");
    }

    x = distance * cos(angle_min + angle_increment*step);
    y = distance * sin(angle_min + angle_increment*step);
}*/

// get index of minimum value

float inverse_meas_bres(int M, int N, float x, float y, float theta, float r, float r_max, float r_min, vector <int> &vector_x, vector <int> &vector_y, vector <float> &vector_prob)
{
    //Steps: 
    // Get Range, bearing
    // convert to global coordinates
    //Run occupancy grid bayes filter
    int x0 = max(1,min(M,round(x)));
    int y0 = max(1,min(N,round(y)));

    int endpt_x = x + r*cos(theta);
    int endpt_y = y + r*sin(theta); 

    int x1 = max(1,min(M,round(endpt_x)));
    int y1 = max(1,min(N,round(endpt_y)));

    bresenham(x0, yo, x1, y1, vector_x, vector_y); 
    for(int i = 0; i < vector_x.size(); i++)
    {
        if(i = vector_x.size() - 1 && r < r_max && r > r_min)
        {
            vector_prob.insert(i, cell_occ); 
        }
        else
        {
            vector_prob.insert(i, cell_not_occ); 
        }
    }   

}

void mapping()
{
    //GO THROUGH EACH CELL
    // CHECK IF CELL's (thorugh relative measurements) are in bearing field
    float prior = logit(0.5);
    int map_size = sizeof(map)/sizeof(map[0]);
    int measurement_size = (angle_max - angle_min)/angle_increment;
    int range_size = sizeof(ranges)/sizeof(ranges[0]);
    MatrixXd measL = MatrixXd::Zero(M,N);


    // find which cells to update
    for(int i = 0; i < measurement_size; i++)
    {
        std::vector <int> vector_x;
        std::vector <int> vector_y;
        std::vector <float> vector_prob;
        float theta = i*angle_increment + ips_yaw;

        inversescannerbres(M,N,ips_x,ips_y,theta,r[i],range_max, range_min, vector_x, vector_y, vector_prob);  // returns points AND their probabilities (0-1)
        for(int j = 0; j < vector_x.size(); j++)
        {
            int ix = vector_x.at(j);
            int iy = vector_y.at(j); 
            float il = vector_prob.at(j); 

            //update log odds
            L(ix,iy) = L(ix,iy) + log(il./(1-il))-L0(ix,iy);
            measL(ix,iy) = measL(ix,iy) + log(il./(1-il)) - L0(ix,iy);
        }
 
    }

    //calculate probabilities
    m = (L.array().exp()).cwiseQuotient(1+L.array().exp());
    invmod_T = (measL.array().exp()).cwiseQuotient(1+measL.array().exp());

    //clear data from previous inputs
    vector_x.clear();
    vector_y.clear();
    vector_prob.clear();
    r.clear(); 

          
}

short sgn(int x) { return x >= 0 ? 1 : -1; }

void laser_callback(const sensor_msgs::LaserScan &msg)
{
    const int RANGE_SIZE = sizeof(msg.ranges);
    msg.angle_min = angle_min; 
    msg.angle_max = angle_max; 
    msg.angle_increment = angle_increment;

    for(int i = 0; i < RANGE_SIZE; i++)
    {
        r.push_back(msg.range[i]); 
    }

}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg) {

    int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

    ips_x = msg.pose[i].position.x;
    ips_y = msg.pose[i].position.y;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

    //store robot poses for plotting path
    robot_pose_x.push_back(ips_x); 
    robot_pose_y.push_back(ips_y);
    robot_pose_yaw.push_back(ips_yaw); 
}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid &msg) {
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s)
            y0 += sgn(dy2);
        else
            x0 += sgn(dx2);
        if (d < 0)
            d += inc1;
        else {
            d += inc2;
            if (s)
                x0 += sgn(dx2);
            else
                y0 += sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

void init_final_occupancy_grid(nav_msgs::OccupancyGrid &msg)
{
    msg.header.frame_id = "map"
    msg.info.resolution  = 0.1 // each cell is 0.1m [m/cell]
    msg.info.width = M; // in cells
    msp.info.height = N; // in cells
    geometry_msgs/Pose pose = [0, 0, 0]; 
    msg.info.origin = pose; // [m, m, rad]

    MatrixXd final_m = 100 * m; // convert probability to be between 0 and 100
    // form data according to ROS specs
    for(int j = 0; j < M; j++)
    {
        for(int k = 0; k < N; k++)
        {
            if(final_map[i][j]) == 50.0)
            {
                final_map[i][j] = -1; //unknown value, ROS settings
            }

        }
         
    }

    msg.data = int(final_map);

    r.clear();  
}

int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    //ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, laser_callback); 

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true); 

    //Velocity control variable
    geometry_msgs::Twist vel;
    nav_msgs::OccupancyGrid occ_grid; 

    //Set the loop rate
    ros::Rate loop_rate(20); //20Hz update rate

    while (ros::ok()) {
        //Main loop code goes here:
        vel.linear.x = 0.1;  // set linear speed
        vel.angular.z = 0.3; // set angular speed

        mapping(); // update map
        init_final_occupancy_grid(occ_grid); // generate occupancy grid message

        velocity_publisher.publish(vel); // Publish the command velocity
        map_publisher.publish(occ_grid);  //publish grid to /map topic
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    }

    return 0;
}
