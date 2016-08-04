// A sample C++ code for assignment #3 - Part II
// This is a simple motion planning program to move robot arms using RRT

#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
#define MAX_ARRAY_SIZE	100
#define LARGE_NUMBER	10000
#define PI	3.141592
#define VERTEX 5000

#include <ros/ros.h>

#include <iostream>     			// to use: std::cout, std::cin, etc
#include <cmath>					// to use: atan2, sqrt, pow, etc
#include <iomanip>     				// to use: setprecision()
#include <geometry_msgs/Twist.h> 	
#include <nav_msgs/Odometry.h>
#include <vector>
#include <limits.h>
#include <ctime>
#include <cstdlib>
#include <sstream>
#include <string>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h> 
#include <trajectory_msgs/JointTrajectory.h> 

// FCL, Boost libraries
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/collision.h>
#include "fcl/collision_node.h"
#include <fcl/distance.h>
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
//#include "test_fcl_utility.h"
#include "fcl/BVH/BVH_model.h"
#include <fcl/BV/OBBRSS.h>
#include "fcl/BV/BV.h"
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>


// ROS/tf libraries
#include <tf/transform_listener.h>		// to use: tf listner
#include <tf/transform_broadcaster.h>	// to use: tf broadcaster
#include <tf/transform_datatypes.h>		// to use: tf datatypes

// a few empty strings for later use...
std::string base_footprint;
std::string cmd_vel;
std::string odom;


// 2-tuple (x, y)
struct Points2D {
 Points2D( double x, double y):_x(x),_y(y)
 {
 }
 double _x , _y;
};

// 3-tuple (SE(2))
struct Points3D {
 Points3D( double x, double y, double z):_x(x),_y(y),_z(z)
 {
 }
 double _x , _y, _z;
};

// 5-tuple (T^6)
struct Points5D {
 Points5D( double q1, double q2, double q3, double q4, double q5):_q1(q1),_q2(q2),_q3(q3),_q4(q4),_q5(q5)
 {
 }
 double _q1 , _q2, _q3, _q4, _q5;
};

// "motionPlanning" class
class motionPlanning
{
	// public functions, variables
	public:

		// constructor
		motionPlanning(ros::NodeHandle& n);
		



		// destructor
		~motionPlanning()
		{
		}
		
		void callbackJoint (const sensor_msgs::JointState::ConstPtr& state);

		// set youbot pose and transformation
		void setYoubot();
		
		// get youbot current pose
		void getYoubotPose();
		
		// draw path in rviz
 		void drawPath();
		
		// smallest difference between two angles
		double angleDiff(double ang1, double ang2);
		
		// collision checking between mobile base-obstacles
		bool collisionCheck(double, double, double);
		
		// continuous collision check between mobile base-obstacles
		bool contcollisionCheck(std::vector<Points3D>,std::vector<Points3D>, double );	

		// collision checking between youbot arms-obstacles
		bool collisionCheckYoubot(double pos[], double th[]);
		
		// continuous collision checking between youbot arms-obstacles
		bool contcollisionCheckYoubot(std::vector<Points5D> pnt_q1,std::vector<Points5D> pnt_q2, double pos1[], double pos2[], double n_sample);
		
		// Euclidean distance between two points in R^3
		double distPointPoint(std::vector<Points3D> v1, std::vector<Points3D> v2);
		
		// minimum distance from two vertices (for dijkstra's algorithm)
		int minDistance(int dist[], bool sptSet[], int v_size);
		
		// smallest distance between a point and multiple points in 3D
		int distPointPoints(std::vector<Points3D>, std::vector<Points3D>);
		
		// dijkstra's algorithm
		std::vector<int> dijkstra(int ** , int, int, int);

		// build RRT, find shortest path in RRT between two configurations in SE(2)
		std::vector<Points3D> simpleRRT1(std::vector<Points3D>, std::vector<Points3D>);
		
		// build RRT, find shortest path in RRT between two configurations in T^5
		std::vector<Points5D> simpleRRT2(std::vector<Points5D> q0, std::vector<Points5D> q1, double ang_max[], double ang_min[]);

		// distance between two configurations in SE(2) using embedding function
		double distEmbedding(std::vector<Points3D> q1, std::vector<Points3D> q2);
		
		// distance between 1 vs many configurations in SE(2) using embedding function
		double distEmbeddings(std::vector<Points3D> q1, std::vector<Points3D> q2);
		
		// get joint positions of youBot
		std::vector<fcl::Vec3f> getJointPositions(std::vector<Points5D> th_r, double pos[]);
		
		// distance between two configurations in T^5 using embedding function
		double distEmbeddingMult(std::vector<Points5D> q1, std::vector<Points5D> q2,double pos1[], double pos2[]);

		// distance between 1 vs many configurations in T^5 using embedding function
		double distEmbeddingMults(std::vector<Points5D> pnt, std::vector<Points5D> poly,  double pos1[], double pos2[]);
		
		// dot product
		double dotP(std::vector<Points3D> v1, std::vector<Points3D> v2);
		
		// generate way points and generate PID control effort
		geometry_msgs::Twist wayPoint(std::vector<Points3D> wp, unsigned int cnt, double kp, double ki, double kd,
		double kp_t, double ki_t, double kd_t);

	private:

		// variables needed to generated PID control effort
		double err_x, err_y, err_th;
		
		double err_acum_x, err_acum_y, err_acum_th;

		// vector for current joint positions
		trajectory_msgs::JointTrajectoryPoint start_pt;
		
		// vector for desired joint positions
		trajectory_msgs::JointTrajectoryPoint end_pt;
		
		// joint trajectory object
		trajectory_msgs::JointTrajectory traj;

		// create a subscriber object;
        ros::Subscriber sub;
		
		// create a publisher object;	
        ros::Publisher pub1, pub2, pub3;

		// create a transfor listener object
        tf::TransformListener listener;
		
		// pose of youbot's origin w.r.t. "odom" coordinate 
		double tf_yb_origin_x, tf_yb_origin_y, tf_yb_origin_yaw;
		
		
		std::vector<Points3D> goal;
		
		// vector to hold waypoints generated with simpleRRT2
		std::vector<Points5D> rrt_path;		
		
		// place 10 intermediate points between every two adjacent waypoints in rrt_path
		std::vector<Points5D> rrt_path_new;
		
		// flag which is set true when callback function receives current joint positions
		bool got_currentposition;

		// youbot's origin w.r.t. "base_footprint"
		tf::Stamped<tf::Pose> yb_origin;
		
		// youbot's origin w.r.t. "odom"
		tf::Stamped<tf::Pose> tf_yb_origin;
		       
};

motionPlanning::motionPlanning(ros::NodeHandle& n)
{
	geometry_msgs::Twist vel;
			
	got_currentposition = false;
	
	// tolerance
	double tol = 0.05;
	
	// PID gains for transitional velocities
	const double kp = 1;
	const double ki = 0.001;
	const double kd = 10; 

	// PID gains for rotational velocities
	const double kp_t = 1;
	const double ki_t = 0.001;
	const double kd_t = 10; 	
	
	unsigned int counter = 0;
	
	// create a subscriber which receives message from "joint_stages"
	sub = n.subscribe("/joint_states", 100, &motionPlanning::callbackJoint,this);

	// assign to "pub" and advertise that we are going to publish msgs to cmd_vel with queue size of 10
	pub1 = n.advertise<geometry_msgs::Twist>(cmd_vel,10);
	
	// assign to "pub" and adverties that we are going to publish msgs to "/arm_1/arm_controller/command"
	pub2 = n.advertise<trajectory_msgs::JointTrajectory>("/arm_1/arm_controller/command",10);

	// create publisher for visualization makers in rviz
	pub3 = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);


	// define rate in ros compatible way
	ros::Rate loop_rate(100);
	
	// set youbot pose, transformation
	setYoubot();
	
	unsigned int choice = 1;
	std::cout << "=====================================" << std::endl;		
	std::cout << "1: path planner (RRT) for mobile base" << std::endl;
	std::cout << "2: path planner (RRT) for youBot arms" << std::endl;		
	std::cout << "Make a choice, type 1 or 2, and press Enter" << std::endl;
	
	std::cin >> choice;
	switch(choice){
		case 1:
		{
			double q1_x, q1_y, q1_z;
			// get user input for goal positions in (x,y)
			std::cout << "Enter your goal (e.g., 5 5 0): ";
			std::cin >> q1_x >> q1_y >> q1_z;

			ros::spinOnce();
			getYoubotPose();
			
			std::vector<Points3D> q0, q1; 
			
			// set initial and goal configurations in SE(2)
			q0.push_back(Points3D(tf_yb_origin_x, tf_yb_origin_y,tf_yb_origin_yaw));	
			q1.push_back(Points3D(q1_x, q1_y, q1_z));
			
			// build RRT and find shortest path on it
			goal = simpleRRT1(q0, q1);
			
			// draw the shortest path in rviz simulator
			drawPath();
			
			// display the shortest path
			std::cout << "==========================" << std::endl;
			for (size_t i = 0 ; i < goal.size(); i++){
				std::cout << i <<": (" << goal[i]._x << ", " << goal[i]._y << ", "<< goal[i]._z <<")" << std::endl;
			}		
			std::cout << "==========================" << std::endl;

			
			while(n.ok()){
				// listening to messages
				ros::spinOnce();	
				
				// get youbot's position (which was received by the listener)
				getYoubotPose();

				
				// if robot is close enough to desired goal, do
				if (std::abs(goal[counter]._x - tf_yb_origin_x) < tol && std::abs(goal[counter]._y - tf_yb_origin_y) < tol &&
				std::abs(atan2(sin(goal[counter]._z - tf_yb_origin_yaw),cos(goal[counter]._z - tf_yb_origin_yaw))) < tol)
				{	
					// if reached at goal configuration
					if (counter == (goal.size()-1))
					{
						// stop the robot
						vel.linear.x = 0.0;
						vel.linear.y = 0.0;
						vel.angular.z = 0.0;
						pub1.publish(vel);
						// ask for another user input for a new goal
						std::cout << std::endl;
						std::cout << "Enter your goal: ";
						std::cin >> q1_x >> q1_y >> q1_z;
						q1.clear();
						q0.clear();
						q0.push_back(Points3D(tf_yb_origin_x, tf_yb_origin_y,tf_yb_origin_yaw));	
						q1.push_back(Points3D(q1_x, q1_y, q1_z));
						goal = simpleRRT1(q0, q1);
						drawPath();				
						
						std::cout << "==========================" << std::endl;
						for (size_t i = 0 ; i < goal.size(); i++){
							std::cout << i <<": (" << goal[i]._x << ", " << goal[i]._y << ", " << goal[i]._z << ")" << std::endl;
						}
						std::cout << "==========================" << std::endl;
						counter = 0;									
						counter +=1;
						err_acum_x = 0;
						err_acum_y = 0;
						err_acum_th = 0;
						err_x = 0;
						err_y = 0;
						err_th = 0;		
						std::cout << "current counter is: " << counter << " / " << goal.size()-1 << std::endl;		
					}
					else
					{
						counter +=1;
						err_acum_x = 0;
						err_acum_y = 0;
						err_acum_th = 0;
						err_x = 0;
						err_y = 0;
						err_th = 0;
						std::cout << "current counter is: " << counter << " / " << goal.size()-1 << std::endl;
					}
				}
				
				vel = wayPoint(goal,counter,kp, ki, kd,kp_t, ki_t,kd_t);
				
				// publish data to "cmd_vel" topic
				pub1.publish(vel);
				
				// sleep at the defined rate (20Hz)
				loop_rate.sleep();
			}		
			break;
		}
		
		case 2:
		{
			double pos[3] = {0, 0, 0};

			// create vectors for initial and goal positions
			std::vector<Points5D> init_pos;
			std::vector<Points5D> goal_pos;
			
			// assign values to created vectors
			init_pos.push_back(Points5D(0, 0, 0, 0, 0));
			goal_pos.push_back(Points5D(0, PI/5, PI/1.6, -PI/3, 0));

			// set joint limits
			double ang_min[5] = {-PI / 180 * 169, -PI / 180 * 65, -PI / 180 * 151 , -PI / 180 * 102.5 , -PI / 180 * 165};
			double ang_max[5] = {PI / 180 * 169, PI / 180 * 90, PI / 180 * 146 , PI / 180 * 102.5 , PI / 180 * 165};

			// initilize traj (joint trajectory) object
			traj.header.stamp = ros::Time::now();
			traj.header.frame_id = base_footprint;
			traj.joint_names.push_back("arm_joint_1");
			traj.joint_names.push_back("arm_joint_2");
			traj.joint_names.push_back("arm_joint_3");
			traj.joint_names.push_back("arm_joint_4");
			traj.joint_names.push_back("arm_joint_5");
			
			end_pt.positions.resize(5);
			
			end_pt.positions[0] = 2.95 + init_pos[0]._q1;
			end_pt.positions[1] = 1.13 + init_pos[0]._q2;
			end_pt.positions[2] = -2.55+ init_pos[0]._q3;
			end_pt.positions[3] = 1.79 + init_pos[0]._q4;
			end_pt.positions[4] = 2.879+ init_pos[0]._q5;

			traj.points.resize(1);
			traj.points[0]=end_pt;
			traj.points[0].time_from_start=ros::Duration(1.0);
			
			double err = 1;
			pub2.publish(traj);
			
			// initialize arms
			while(err > 0.01)
			{
				ros::spinOnce();	
				
				if (got_currentposition)
				{
					err = 0;
					for (size_t i=0; i<5;i++)
					{
						err+=std::abs(angleDiff(start_pt.positions[i],end_pt.positions[i]));
					}
					got_currentposition = false;
				}
				pub2.publish(traj);
				loop_rate.sleep();
			}
			
			// display messeges
			std::cout << "initialized at (" << start_pt.positions[0]-2.95 << ", " <<
			start_pt.positions[1] - 1.13 << ", " << start_pt.positions[2]+2.55  << ", " <<
			start_pt.positions[3] - 1.79 << ", " << start_pt.positions[4]-2.879 <<") !" << std::endl;

			// generate RRT and shortest path in RRT
			rrt_path = simpleRRT2(init_pos, goal_pos, ang_min, ang_max);

			// create way points
			for (size_t m = 0; m < rrt_path.size()-1; m++)
			{
				for (size_t j = 0; j < 20; j++)
				{
					rrt_path_new.push_back(Points5D(rrt_path[m]._q1 + j * (rrt_path[m+1]._q1 - rrt_path[m+1]._q1)/20,
					rrt_path[m]._q2 + j * (rrt_path[m+1]._q2 - rrt_path[m+1]._q2)/20,
					rrt_path[m]._q3 + j * (rrt_path[m+1]._q3 - rrt_path[m+1]._q3)/20,
					rrt_path[m]._q4 + j * (rrt_path[m+1]._q4 - rrt_path[m+1]._q4)/20,
					rrt_path[m]._q5 + j * (rrt_path[m+1]._q5 - rrt_path[m+1]._q5)/20));
				}
			}
			rrt_path_new.push_back(Points5D(rrt_path[rrt_path.size()-1]._q1,
			rrt_path[rrt_path.size()-1]._q2,
			rrt_path[rrt_path.size()-1]._q3,
			rrt_path[rrt_path.size()-1]._q4,
			rrt_path[rrt_path.size()-1]._q5));
			
			ros::spinOnce();
			getYoubotPose();
			
			double place_holder[5];

			// display the shortest path
			std::cout << "==========================" << std::endl;	
			std::cout << "Your RRT path is:" << std::endl;
			for (size_t i = 0; i < rrt_path.size(); i++)
			{
				std::cout << i << " : " << rrt_path[i]._q1 << ", " << rrt_path[i]._q2 << ", " 
				<< rrt_path[i]._q3 << ", " << rrt_path[i]._q4 << ", " << rrt_path[i]._q5 << std::endl;
			}
			std::cout << "==========================" << std::endl;	
			std::cout << "moving arms to goal ";

			// set desired joint positions
			end_pt.positions[0] = 2.95 + rrt_path_new[counter]._q1;
			end_pt.positions[1] = 1.13 + rrt_path_new[counter]._q2;
			end_pt.positions[2] = -2.55+ rrt_path_new[counter]._q3;
			end_pt.positions[3] = 1.79 + rrt_path_new[counter]._q4;
			end_pt.positions[4] = 2.879+ rrt_path_new[counter]._q5;		
			
			while(n.ok()){
				// listening to messages
				ros::spinOnce();	
						
				// evalulate
				if (got_currentposition)
				{
					err = 0;
					// compute joint position error
					for (size_t i=0; i<5;i++)
					{
						err+=std::abs(angleDiff(start_pt.positions[i],end_pt.positions[i]));
					}
					got_currentposition = false;
					
					if (err < 0.01)
					{
						// increase counter
						counter++;
						std::cout << " " << counter;
						if (counter == rrt_path_new.size())
						{
							std::cout << std::endl;
							std::cout << "goal reached!" << std::endl;
							std::cout << "enter new goal: (e.g., 0 0.6283 1.9635 -1.0472 0)" << std::endl;
							init_pos.clear();
							// set current position as the previous goal position (approximation)
							init_pos.push_back(Points5D(rrt_path_new[counter-1]._q1,rrt_path_new[counter-1]._q2,rrt_path_new[counter-1]._q3,rrt_path_new[counter-1]._q4,rrt_path_new[counter-1]._q5));
							goal_pos.clear();
							// get user input: new joint posiions
							std::cin >> place_holder[0] >> place_holder[1] >> place_holder[2] >> place_holder[3] >> place_holder[4];  
							goal_pos.push_back(Points5D(place_holder[0], place_holder[1], place_holder[2], place_holder[3], place_holder[4]));	
							// clear some vectors
							rrt_path.clear();
							rrt_path_new.clear();
							// generate RRT and shortest path in RRT
							rrt_path = simpleRRT2(init_pos, goal_pos, ang_min, ang_max);
							
							// create waypoints
							for (size_t m = 0; m < rrt_path.size()-1; m++)
							{
								for (size_t j = 0; j < 20; j++)
								{
									rrt_path_new.push_back(Points5D(rrt_path[m]._q1 + j * (rrt_path[m+1]._q1 - rrt_path[m+1]._q1)/20,
									rrt_path[m]._q2 + j * (rrt_path[m+1]._q2 - rrt_path[m+1]._q2)/20,
									rrt_path[m]._q3 + j * (rrt_path[m+1]._q3 - rrt_path[m+1]._q3)/20,
									rrt_path[m]._q4 + j * (rrt_path[m+1]._q4 - rrt_path[m+1]._q4)/20,
									rrt_path[m]._q5 + j * (rrt_path[m+1]._q5 - rrt_path[m+1]._q5)/20));
								}
							}
							rrt_path_new.push_back(Points5D(rrt_path[rrt_path.size()-1]._q1,
							rrt_path[rrt_path.size()-1]._q2,
							rrt_path[rrt_path.size()-1]._q3,
							rrt_path[rrt_path.size()-1]._q4,
							rrt_path[rrt_path.size()-1]._q5));													
							
							counter = 0;
							std::cout << "==========================" << std::endl;
							std::cout << "Your RRT path is:" << std::endl;
							for (size_t i = 0; i < rrt_path.size(); i++)
							{
								std::cout << i << " : " << rrt_path[i]._q1 << ", " << rrt_path[i]._q2 << ", " 
								<< rrt_path[i]._q3 << ", " << rrt_path[i]._q4 << ", " << rrt_path[i]._q5 << std::endl;
							}
							std::cout << "==========================" << std::endl;
							
							std::cout << "moving arms to goal ";
						}
					}
				}
				// set desired joint positions
				end_pt.positions[0] = 2.95 + rrt_path_new[counter]._q1;
				end_pt.positions[1] = 1.13 + rrt_path_new[counter]._q2;
				end_pt.positions[2] = -2.55+ rrt_path_new[counter]._q3;
				end_pt.positions[3] = 1.79 + rrt_path_new[counter]._q4;
				end_pt.positions[4] = 2.879+ rrt_path_new[counter]._q5;		

				traj.header.stamp = ros::Time::now();
				traj.points.resize(1);
				traj.points[0]=end_pt;
				traj.points[0].time_from_start=ros::Duration(1.0);

				// update trajectory
				pub2.publish(traj);		
				
				// get youbot's position (which was received by the listener)
				getYoubotPose();
				// sleep at the defined rate
				loop_rate.sleep();
			}
			break;
		}

	}
	

	
}

// draw path in rviz
void motionPlanning::drawPath()
{
	visualization_msgs::Marker points, line_strip, line_list;
	points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = odom;
	points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
	points.ns = line_strip.ns = line_list.ns = "sample2_node";
	points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

	points.id = 0;
	line_strip.id = 1;
	line_list.id = 2;

	points.type = visualization_msgs::Marker::POINTS;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_list.type = visualization_msgs::Marker::LINE_LIST;

	// POINTS markers use x and y scale for width/height respectively
	points.scale.x = 0.05;
	points.scale.y = 0.05;

	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	line_strip.scale.x = 0.1;
	line_list.scale.x = 0.1;

	// points are green
	points.color.g = 1.0f;
	points.color.a = 1.0;

	// line strip is blue
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;

	// line list is red
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	for (size_t i = 0 ; i < goal.size(); i++){
		geometry_msgs::Point p;
		p.x = goal[i]._x;
		p.y = goal[i]._y;
		p.z = 0;
		points.points.push_back(p);
		line_strip.points.push_back(p);	
	}
	// create the vertices for the points and lines
	pub3.publish(points);
	pub3.publish(line_strip);
}

// distance between two configurations in SE(2) using embedding function
double motionPlanning::distEmbedding(std::vector<Points3D> q1, std::vector<Points3D> q2)
{
	double ctl_pnt1[2] = {0.3, 0.2};
	double ctl_pnt2[2] = {-0.3, 0.2};
	double cth1 = cos(q1[0]._z);
	double sth1 = sin(q1[0]._z);
	double cth2 = cos(q2[0]._z);
	double sth2 = sin(q2[0]._z);
	std::vector<Points3D> q11_w, q12_w, q21_w, q22_w;
	q11_w.push_back(Points3D(cth1 * ctl_pnt1[0] - sth1* ctl_pnt1[1] + q1[0]._x, sth1 * ctl_pnt1[0] + cth1 * ctl_pnt1[1] + q1[0]._y,0));
	q12_w.push_back(Points3D(cth1 * ctl_pnt2[0] - sth1* ctl_pnt2[1] + q1[0]._x, sth1 * ctl_pnt2[0] + cth1 * ctl_pnt2[1] + q1[0]._y,0));
	q21_w.push_back(Points3D(cth2 * ctl_pnt1[0] - sth2* ctl_pnt1[1] + q2[0]._x, sth2 * ctl_pnt1[0] + cth2 * ctl_pnt1[1] + q2[0]._y,0));
	q22_w.push_back(Points3D(cth2 * ctl_pnt2[0] - sth2* ctl_pnt2[1] + q2[0]._x, sth2 * ctl_pnt2[0] + cth2 * ctl_pnt2[1] + q2[0]._y,0));
	
	return sqrt(
	pow(distPointPoint(q11_w,q21_w),2) + 
	pow(distPointPoint(q12_w,q22_w),2)
	);
}

// get joint positions of youBot
std::vector<fcl::Vec3f> motionPlanning::getJointPositions(std::vector<Points5D> th_r, double pos[])
{
	// D-H parameters
	double th[] = {0,th_r[0]._q1,th_r[0]._q2-PI/2,th_r[0]._q3,th_r[0]._q4,PI/2,th_r[0]._q5+PI/2};
	double a[] = {0.167, 0.033, 0.155, 0.135, 0.1136, 0, 0};
	double ap[] = {PI, PI/2, 0, 0, -PI/2, PI/2, 0};
	double d[] = {0.245, 0, 0, 0, 0, 0, 0.05716};

	double yb_org_x = pos[0];
	double yb_org_y = pos[1];
	double yb_org_yaw = pos[2];

	std::vector<fcl::Matrix3f> rot;
	std::vector<fcl::Vec3f> tr;
	std::vector<fcl::Vec3f> ee;
	fcl::Vec3f k(0,0,0);

	std::vector<fcl::Transform3f> A;
	std::vector<fcl::Transform3f> T;

	// mobile base
	std::vector<fcl::Matrix3f> rot_base;
	std::vector<fcl::Vec3f> tr_base;	
	std::vector<fcl::Transform3f> A_base;
	
	rot_base.push_back(fcl::Matrix3f(cos(yb_org_yaw),-sin(yb_org_yaw),0,
	sin(yb_org_yaw), cos(yb_org_yaw),0,
	0,0,1));	
	tr_base.push_back(fcl::Vec3f(yb_org_x,yb_org_y,0));
	A_base.push_back(fcl::Transform3f(rot_base[0],tr_base[0]));
	
	// compute homogenous transformation matrices
	for (size_t i=0; i < 7; ++i)
	{
		rot.push_back(fcl::Matrix3f(cos(th[i]),-sin(th[i])*cos(ap[i]),sin(th[i])*sin(ap[i]),
		sin(th[i]), cos(th[i])*cos(ap[i]),-cos(th[i])*sin(ap[i]),
		0,sin(ap[i]),cos(ap[i])));

		tr.push_back(fcl::Vec3f(a[i]*cos(th[i]),a[i]*sin(th[i]),d[i]));

		A.push_back(fcl::Transform3f(rot[i],tr[i]));
	}
	for (size_t i=0; i != A.size(); ++i)
	{
		if (i > 0){
			T.push_back(T[i-1]*A[i]);
		}
		else{
			T.push_back(A_base[0]*A[i]);
		}
		ee.push_back(T[i].transform(k));
	}
	
	// return joint positions
	return ee;
}

// distance between two configurations in T^5 using embedding function
double motionPlanning::distEmbeddingMult(std::vector<Points5D> q1, std::vector<Points5D> q2, double pos1[], double pos2[])
{
	std::vector<fcl::Vec3f> ee1 = getJointPositions(q1, pos1);
	std::vector<fcl::Vec3f> ee2 = getJointPositions(q2, pos2);
	double err=0;
	for (size_t j=0;j<5;j++)
	{
		err+=sqrt(pow(ee1[j][0]-ee2[j][0],2) + pow(ee1[j][1]-ee2[j][1],2) + pow(ee1[j][2]-ee2[j][2],2));
	}
	return err;
}

// distance between 1 vs many configurations in T^5 using embedding function
double motionPlanning::distEmbeddingMults(std::vector<Points5D> pnt, std::vector<Points5D> poly,  double pos1[], double pos2[])
{
	// find index in poly which makes the distance between pnt and poly the minimum
	std::vector<double> output;
	std::vector<double> min_val;
	double min_val_so_far = 1000;
	double min_idx_so_far;
	for (size_t i = 0; i != poly.size()-1; ++i){
		std::vector<Points5D> cur_pnt;
		double tmp_str;
		cur_pnt.push_back(Points5D(poly[i]._q1, poly[i]._q2, poly[i]._q3, poly[i]._q4, poly[i]._q5));
		tmp_str = distEmbeddingMult(pnt, cur_pnt, pos1, pos2);
		
		min_val.push_back(tmp_str);
		if (min_val[i] < min_val_so_far){
			min_val_so_far = min_val[i];
			min_idx_so_far = i;
		}
	}
	return min_idx_so_far;	
}

// distance between 1 vs many configurations in SE(2) using embedding function
double motionPlanning::distEmbeddings(std::vector<Points3D> pnt, std::vector<Points3D> poly)
{
	// find index in poly which makes the distance between pnt and poly the minimum
	std::vector<double> output;
	std::vector<double> min_val;

	double min_val_so_far = 1000;
	int min_idx_so_far;
		
	for (size_t i = 0; i != poly.size()-1; ++i){
		std::vector<Points3D> cur_pnt;
		double tmp_str;
		cur_pnt.push_back(Points3D(poly[i]._x, poly[i]._y, poly[i]._z));
		tmp_str = distEmbedding(pnt, cur_pnt);
		min_val.push_back(tmp_str);
		if (min_val[i] < min_val_so_far){
			min_val_so_far = min_val[i];
			min_idx_so_far = i;
		}
	}
	return min_idx_so_far;	
}

// collision checking between mobile base-obstacles
bool motionPlanning::collisionCheck(double yb_pos_x, double yb_pos_y, double yb_pos_yaw)
{
	boost::shared_ptr<fcl::Box> D_yb(new fcl::Box(0.6,0.4,0.6));
	boost::shared_ptr<fcl::Box> D_obs_1(new fcl::Box(2,2,1));
	boost::shared_ptr<fcl::Box> D_obs_2(new fcl::Box(0.75,0.75,0.75));
	boost::shared_ptr<fcl::Box> D_obs_3(new fcl::Box(0.75,0.75,0.75));
	boost::shared_ptr<fcl::Box> D_obs_4(new fcl::Box(0.5,0.5,0.5));
	boost::shared_ptr<fcl::Box> D_obs_5(new fcl::Box(0.4,0.4,0.2));
	boost::shared_ptr<fcl::Box> D_wall_1(new fcl::Box(8,0.5,0.5));
	boost::shared_ptr<fcl::Box> D_wall_2(new fcl::Box(0.5,8,0.5));
	boost::shared_ptr<fcl::Box> D_wall_3(new fcl::Box(8,0.5,0.5));
	boost::shared_ptr<fcl::Box> D_wall_4(new fcl::Box(0.5,8,0.5));			


	fcl::Transform3f D_tf_yb, D_tf_obs_1, D_tf_obs_2, D_tf_obs_3, D_tf_obs_4, D_tf_obs_5, D_tf_wall_1, D_tf_wall_2, D_tf_wall_3, D_tf_wall_4;

	D_tf_yb.setIdentity();
	D_tf_obs_1.setIdentity();
	D_tf_obs_2.setIdentity();
	D_tf_obs_3.setIdentity();
	D_tf_obs_4.setIdentity();
	D_tf_obs_5.setIdentity();
	D_tf_wall_1.setIdentity();
	D_tf_wall_2.setIdentity();
	D_tf_wall_3.setIdentity();
	D_tf_wall_4.setIdentity();	
	double q[3] = {yb_pos_x, yb_pos_y, yb_pos_yaw};
	D_tf_yb.setRotation(fcl::Matrix3f(cos(q[2]),-sin(q[2]),0, sin(q[2]), cos(q[2]),0, 0,0,1));
	D_tf_yb.setTranslation(fcl::Vec3f(q[0],q[1], 0.3));
	D_tf_obs_1.setTranslation(fcl::Vec3f(2.5,2.5,0.5));
	D_tf_obs_2.setTranslation(fcl::Vec3f(5,1,0.375));
	D_tf_obs_3.setTranslation(fcl::Vec3f(3,5,0.375));
	D_tf_obs_4.setTranslation(fcl::Vec3f(0,3,0.25));
	D_tf_obs_5.setTranslation(fcl::Vec3f(0.6,0,0.1));
	D_tf_wall_1.setTranslation(fcl::Vec3f(2.5,-1.25,0.25));
	D_tf_wall_2.setTranslation(fcl::Vec3f(6.25,2.5,0.25));
	D_tf_wall_3.setTranslation(fcl::Vec3f(2.5,6.25,0.25));
	D_tf_wall_4.setTranslation(fcl::Vec3f(-1.25,2.5,0.25));	

	fcl::CollisionResult result_1, result_2, result_3, result_4, result_5, result_6, result_7, result_8, result_9, result_10, result_11;
	fcl::CollisionRequest request_1, request_2, request_3, request_4, request_5, request_6, request_7, request_8, request_9, request_10, request_11;


	fcl::CollisionObject co_yb(D_yb, D_tf_yb);
	fcl::CollisionObject co_obs_1(D_obs_1, D_tf_obs_1);
	fcl::CollisionObject co_obs_2(D_obs_2, D_tf_obs_2);
	fcl::CollisionObject co_obs_3(D_obs_3, D_tf_obs_3);
	fcl::CollisionObject co_obs_4(D_obs_4, D_tf_obs_4);
	fcl::CollisionObject co_obs_5(D_obs_5, D_tf_obs_5);
	
	fcl::CollisionObject co_wall_1(D_wall_1, D_tf_wall_1);
	fcl::CollisionObject co_wall_2(D_wall_2, D_tf_wall_2);
	fcl::CollisionObject co_wall_3(D_wall_3, D_tf_wall_3);
	fcl::CollisionObject co_wall_4(D_wall_4, D_tf_wall_4);

	fcl::collide(&co_yb, &co_obs_1, request_1, result_1);
	fcl::collide(&co_yb, &co_obs_2, request_2, result_2);
	fcl::collide(&co_yb, &co_obs_3, request_3, result_3);
	fcl::collide(&co_yb, &co_obs_4, request_4, result_4);
	fcl::collide(&co_yb, &co_obs_5, request_11, result_11);
	
	fcl::collide(&co_yb, &co_wall_1, request_7, result_7);
	fcl::collide(&co_yb, &co_wall_2, request_8, result_8);
	fcl::collide(&co_yb, &co_wall_3, request_9, result_9);
	fcl::collide(&co_yb, &co_wall_4, request_10, result_10);
	

	if (!result_1.isCollision() && !result_2.isCollision() && !result_3.isCollision() && !result_4.isCollision() && 
	!result_7.isCollision() && !result_8.isCollision() && !result_9.isCollision() && !result_10.isCollision() && !result_11.isCollision())
	{
		return false;
	}
	else
	{
		return true;
	}
}

// continuous collision check between mobile base-obstacles
bool motionPlanning::contcollisionCheck(std::vector<Points3D> pnt_q1,std::vector<Points3D> pnt_q2, double n_sample)
{
	double q1[3], q2[3];
	q1[0] = pnt_q1[0]._x;
	q1[1] = pnt_q1[0]._y;
	q1[2] = pnt_q1[0]._z;
	q2[0] = pnt_q2[0]._x;
	q2[1] = pnt_q2[0]._y;
	q2[2] = pnt_q2[0]._z;
	bool comp_val = true;
	for (size_t i=0; i < n_sample+1; i++)
	{
		comp_val*=!collisionCheck(q1[0]+i*(q2[0]-q1[0])/n_sample,q1[1]+i*(q2[1]-q1[1])/n_sample,q1[2]+i*(q2[2]-q1[2])/n_sample);
	}	
	return !comp_val;
}


int motionPlanning::minDistance(int dist[], bool sptSet[], int v_size)
{
    // Initialize min value
    int min = INT_MAX, min_index;
 
    for (int v = 0; v < v_size; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;
 
    return min_index;
}
 
// Index of poly which makes the distance between pnt and poly minimum
int motionPlanning::distPointPoints(std::vector<Points3D> pnt, std::vector<Points3D> poly)
{
	// some declaration of variables and initialization
	std::vector<double> output;
	std::vector<double> min_val;
	double min_val_so_far = 1000;
	int min_idx_so_far;
		
	for (size_t i = 0; i != poly.size()-1; ++i){
		std::vector<Points3D> cur_pnt;
		double tmp_str;
		cur_pnt.push_back(Points3D(poly[i]._x, poly[i]._y, poly[i]._z));
		tmp_str = distPointPoint(pnt, cur_pnt);
		
		min_val.push_back(tmp_str);
		if (min_val[i] < min_val_so_far){
			min_val_so_far = min_val[i];
			min_idx_so_far = i;
		}
	}
	return min_idx_so_far;
}

// smallest difference between two angles ang2-ang1 (including its sign)
double motionPlanning::angleDiff(double ang1, double ang2)
{
	return atan2(sin(ang2 - ang1),cos(ang2 - ang1));
}
 
// Funtion that implements Dijkstra's single source shortest path
// algorithm for a graph represented using adjacency matrix
// representation
std::vector<int> motionPlanning::dijkstra(int **graph, int src, int snk, int v_size)
{

	//int VERTEX = vertex;
	const int VT = v_size;
    int dist[VT];	// The output array. dist[i] will hold
					// the shortest distance from src to i
    // sptSet[i] will true if vertex i is included / in shortest
    // path tree or shortest distance from src to i is finalized
    bool sptSet[VT];
 
    // Parent array to store shortest path tree
    int parent[VT];
 
    // Initialize all distances as INFINITE and stpSet[] as false
    for (int i = 0; i < VT; i++)
    {
        parent[0] = -1;
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }
 
    // Distance of source vertex from itself is always 0
    dist[src] = 0;
 
    // Find shortest path for all vertices
    for (int count = 0; count < VT-1; count++)
    {
        // Pick the minimum distance vertex from the set of
        // vertices not yet processed. u is always equal to src
        // in first iteration.
        int u = minDistance(dist, sptSet, v_size);
 
        // Mark the picked vertex as processed
        sptSet[u] = true;
 
        // Update dist value of the adjacent vertices of the
        // picked vertex.
        for (int v = 0; v < VT; v++)
 
            // Update dist[v] only if is not in sptSet, there is
            // an edge from u to v, and total weight of path from
            // src to v through u is smaller than current value of
            // dist[v]
            if (!sptSet[v] && graph[u][v] &&
                dist[u] + graph[u][v] < dist[v])
            {
                parent[v]  = u;
                dist[v] = dist[u] + graph[u][v];
            }  
    }
 
    std::vector<int> path;
    std::vector<int> path_r; 

    path.push_back(snk);
	
	// find the shortest path from source to sink
    for (size_t i = 1; i< VT; i++)
    {
		path.push_back(parent[path[i-1]]);
		if (path[i] == -1){
			break;
		}
	}
    for (int j = 0; j< path.size()-1; j++)
    {
		path_r.push_back(path[path.size()-j-2]);
	}
    
    return path_r;
}

// build RRT and find the shortest path on RRT for two configurations in SE(2)
std::vector<Points3D> motionPlanning::simpleRRT1(std::vector<Points3D> q0, std::vector<Points3D> q1)
{
	std::vector<Points3D> Vt, q_rand, q_new; 	
	Vt.push_back(Points3D(q0[0]._x,q0[0]._y,q0[0]._z));
    const double step_size = 0.5;
    const double step_size_th = 0.1;
    double set_x, set_y, set_o;
    const double n_sample = 10;
    unsigned int max_iter = 60000;
    bool exit = false;
    int **graph = NULL;
    
    int grh_sz = 0;
    int min_idx = -1;
	
    bool true_exit = false;
	size_t i = 0;
	// do until q_new == q1 
	while(!true_exit){
		exit = false;
		// find q_rand in O_free
		while(!exit){
			// generate a random sample for each configurations (SE(2))
			set_x = ((double) rand() / (RAND_MAX))*7 - 1;
			set_y = ((double) rand() / (RAND_MAX))*7 - 1;
			set_o = ((double) rand() / (RAND_MAX))*2*PI-PI;
			if (!collisionCheck(set_x,set_y,set_o))
			{
				exit = true;
			}
		}
		q_rand.push_back(Points3D(set_x,set_y,set_o));
		min_idx = distEmbeddings(q_rand,Vt);
		std::vector<Points3D> Vt_tmp;
		Vt_tmp.push_back(Points3D(Vt[min_idx]._x,Vt[min_idx]._y,Vt[min_idx]._z));
		
		std::vector<Points3D> Vt_tmp2d, q_rand2d;
		Vt_tmp2d.push_back(Points3D(Vt[min_idx]._x,Vt[min_idx]._y,0));
		q_rand2d.push_back(Points3D(set_x,set_y,0));
		double q_new_x, q_new_y, q_new_z;
					
		
		// modify q_new based upon the step_size
		if (distPointPoint(Vt_tmp2d,q_rand2d) > step_size)
		{
			q_new_x = Vt_tmp[0]._x + step_size*(q_rand[0]._x - Vt_tmp[0]._x)/distPointPoint(Vt_tmp,q_rand);
			q_new_y = Vt_tmp[0]._y + step_size*(q_rand[0]._y - Vt_tmp[0]._y)/distPointPoint(Vt_tmp,q_rand);
		}
		else
		{
					
	
			q_new_x = q_rand[0]._x;
			q_new_y = q_rand[0]._y;
		}
		
		if (std::abs(atan2(sin(q_rand[0]._z- Vt_tmp[0]._z), cos(q_rand[0]._z- Vt_tmp[0]._z))) > step_size_th)
		{
			double orig_dist = atan2(sin(q_rand[0]._z- Vt_tmp[0]._z), cos(q_rand[0]._z- Vt_tmp[0]._z));
			if (orig_dist >0)
			{
				q_new_z = Vt_tmp[0]._z+ step_size_th;
			}
			else
			{
				q_new_z = Vt_tmp[0]._z- step_size_th;
			}
		}
		else
		{
			q_new_z = q_rand[0]._z;
		}		
		
		q_new.push_back(Points3D(q_new_x,q_new_y,q_new_z));

		
		// check continuous collision between q_new and the closest point on RRT
		if (!contcollisionCheck(Vt_tmp,q_new, n_sample))
		{	
			////////////////////////////////////////////////////////
			if (!(graph == NULL))
			{
				//std::cout << "debug" << " " << tmp_sz1 << std::endl;
				//tmp = graph;
				
				int **tmp = graph; // temporary place holder for previous graph
				
				// initialize new graph with expanded size
				graph = new int*[Vt.size()+1] ();
				
				for (int i1 = 0; i1 < Vt.size()+1; i1++){
					graph[i1] = new int[Vt.size()+1] ();
				}
				
				// copy tmp -> graph
				for (int z1 = 0; z1 < grh_sz; z1++){
					for (int z2 = 0; z2 < grh_sz; z2++){
						//std::cout << "debug" << std::endl;
						graph[z1][z2] = tmp[z1][z2];
					}
				}
				
				// delete tmp graph
				for (int z3 = 0;z3 < grh_sz; ++z3){
					delete[] tmp[z3];
				}			
				delete[] tmp;
			}
			// if the graph is NULL no need to deal with tmp graph
			else
			{
				graph = new int*[Vt.size()+1] ();
				for (int i1 = 0; i1 < Vt.size()+1; ++i1){
					graph[i1] = new int[Vt.size()+1] ();
				}				
			}
			////////////////////////////////////////////////////////
			//std::cout << "debug" << std::endl;
			graph[min_idx][Vt.size()] = 1;
			grh_sz = Vt.size()+1;
			
			
			Vt.push_back(Points3D(q_new[0]._x,q_new[0]._y,q_new[0]._z));
			
			std::vector<Points3D> comp1,comp2;
			comp1.push_back(Points3D(q_new[0]._x, q_new[0]._y, 0));
			comp2.push_back(Points3D(q1[0]._x, q1[0]._y, 0));
			// check if q_new is close enough to q1 (less than step_size)
			if ((distPointPoint(comp1,comp2) < step_size) && (std::abs(atan2(sin(q1[0]._z- q_new[0]._z), cos(q1[0]._z- q_new[0]._z)))< step_size_th) 
			&& !contcollisionCheck(q_new,q1, n_sample))
			{
				// same deal as before: graph -> tmp, 
				// expand graph
				// tmp -> expaned graph
				// delete tmp
				int **tmp1 = graph;
				graph = new int*[Vt.size()+1] ();
				for (int i1 = 0; i1 < Vt.size()+1; i1++){
					graph[i1] = new int[Vt.size()+1] ();
				}
				for (int z1 = 0; z1 < grh_sz; z1++){
					for (int z2 = 0; z2 < grh_sz; z2++){
						graph[z1][z2] = tmp1[z1][z2];
					}
				}
				for (int z3 = 0;z3 < grh_sz; z3++){
					delete[] tmp1[z3];
				}			
				delete[] tmp1;
				
				graph[Vt.size()-1][Vt.size()] = 1;
				grh_sz = Vt.size()+1;
				Vt.push_back(Points3D(q1[0]._x,q1[0]._y,q1[0]._z));
				true_exit = true;
			}
		}
		
		q_new.clear();
		q_rand.clear();
		Vt_tmp.clear();
		q_rand2d.clear();
		Vt_tmp2d.clear();
		
		i+=1;
	}

	// test collision free
	bool collision_free = true;
	for (size_t j =1 ; j < Vt.size() ; j++)
	{
		collision_free *= (!collisionCheck(Vt[i]._x,Vt[i]._y,0));
	}
	
	if (collision_free==true)
	{
		std::cout << "Collision-free path was found in RRT with " << i << " samples!" << std::endl;
	}
	
	// use Dijkstra's algorithm to find the shortest path on RRT
	std::vector<int> path=dijkstra(graph, 0, Vt.size()-1,Vt.size());

	std::vector<Points3D> Vt_new;
	
	for (size_t i = 0 ; i < path.size(); i++){
		Vt_new.push_back(Points3D(Vt[path[i]]._x, Vt[path[i]]._y, Vt[path[i]]._z));
	}	

	for (int z3 = 0;z3 < grh_sz; z3++){
		delete[] graph[z3];
	}			
	delete[] graph;
	
	return Vt_new;	
}

// build RRT and find the shortest path on RRT for two configurations in T^5
std::vector<Points5D> motionPlanning::simpleRRT2(std::vector<Points5D> q0, std::vector<Points5D> q1, double ang_minl[5], double ang_maxl[5])
{
	std::cout << "building RRT" << std::endl;
	
	std::vector<Points5D> Vt, q_rand, q_new;
	
	Vt.push_back(q0[0]);

    const double step_size = 0.75;

    double set_q[5];
    
    const double n_sample = 20;
    unsigned int max_iter = 30000;
    bool exit = false;
    int min_idx;
	int **graph = NULL;
    int grh_sz = 0;

	std::vector<Points5D> Vt_tmp;

	
    bool true_exit = false;
	size_t i = 0;
	double pos[3] = {0, 0, 0};
	// do until q_new == q1 	
	while(!true_exit){
		
		exit = false;
		
		// find q_rand in O_free
		while(!exit){
			// generate a random sample for each configurations (T^5)
			for (size_t j=0; j< 5;j++)
			{
				set_q[j] = ((double) rand() / (RAND_MAX))*(ang_maxl[j] - ang_minl[j])+ang_minl[j];
			}
			set_q[0] = 0;
			set_q[4] = 0;
			if (!collisionCheckYoubot(pos, set_q))
			{
				exit = true;
			}
		}
		q_rand.push_back(Points5D(set_q[0],set_q[1],set_q[2],set_q[3],set_q[4]));		
		min_idx = distEmbeddingMults(q_rand, Vt, pos, pos);
		Vt_tmp.push_back(Points5D(Vt[min_idx]._q1,Vt[min_idx]._q2,Vt[min_idx]._q3,Vt[min_idx]._q4,Vt[min_idx]._q5));
		double q_new_p[5] = {0,0,0,0,0};
		double orig_dist;
				
		// modify q_new based upon the step_size
		if (std::abs(atan2(sin(q_rand[0]._q1- Vt_tmp[0]._q1), cos(q_rand[0]._q1- Vt_tmp[0]._q1))) > step_size)
		{
			if (angleDiff(ang_minl[0], Vt_tmp[0]._q1+ step_size) <0 &&
			angleDiff(Vt_tmp[0]._q1+ step_size, ang_maxl[0]) <0)
			{
				q_new_p[0] = Vt_tmp[0]._q1- step_size;
			}
			else
			{
				q_new_p[0] = Vt_tmp[0]._q1+ step_size;
			}
		}
		else
		{
			q_new_p[0] = q_rand[0]._q1;
		}
					
		if (std::abs(atan2(sin(q_rand[0]._q2- Vt_tmp[0]._q2), cos(q_rand[0]._q2- Vt_tmp[0]._q2))) > step_size)
		{
			if (angleDiff(ang_minl[1], Vt_tmp[0]._q2+ step_size) >0  &&
			angleDiff(Vt_tmp[0]._q2+ step_size, ang_maxl[1]) >0)
			{
				q_new_p[1] = Vt_tmp[0]._q2+ step_size;
			}
			else
			{
				
				
				q_new_p[1] = Vt_tmp[0]._q2- step_size;
			}							
		}
		else
		{
			q_new_p[1] = q_rand[0]._q2;
		}			
		if (std::abs(atan2(sin(q_rand[0]._q3- Vt_tmp[0]._q3), cos(q_rand[0]._q3- Vt_tmp[0]._q3))) > step_size)
		{
			if (angleDiff(ang_minl[2], Vt_tmp[0]._q3+ step_size) <0 &&
			angleDiff(Vt_tmp[0]._q3+ step_size, ang_maxl[2]) <0)
			{
				q_new_p[2] = Vt_tmp[0]._q3- step_size;
			}
			else
			{
				q_new_p[2] = Vt_tmp[0]._q3+ step_size;
			}				
		}
		else
		{
			q_new_p[2] = q_rand[0]._q3;
		}			
		if (std::abs(atan2(sin(q_rand[0]._q4- Vt_tmp[0]._q4), cos(q_rand[0]._q4- Vt_tmp[0]._q4))) > step_size)
		{
			if (angleDiff(ang_minl[3], Vt_tmp[0]._q4+ step_size) <0 &&
			angleDiff(Vt_tmp[0]._q4+ step_size, ang_maxl[3]) <0)
			{
				q_new_p[3] = Vt_tmp[0]._q4- step_size;
			}
			else
			{
				q_new_p[3] = Vt_tmp[0]._q4+ step_size;
			}				
		}
		else
		{
			q_new_p[3] = q_rand[0]._q4;
		}			
		if (std::abs(atan2(sin(q_rand[0]._q5- Vt_tmp[0]._q5), cos(q_rand[0]._q5- Vt_tmp[0]._q5))) > step_size)
		{
			if (angleDiff(ang_minl[4], Vt_tmp[0]._q5+ step_size) <0 &&
			angleDiff(Vt_tmp[0]._q5+ step_size, ang_maxl[4]) <0)
			{
				q_new_p[4] = Vt_tmp[0]._q5- step_size;
			}
			else
			{
				q_new_p[4] = Vt_tmp[0]._q5+ step_size;
			}						
		}
		else
		{
			q_new_p[4] = q_rand[0]._q5;
		}
									
		q_new.push_back(Points5D(q_new_p[0],q_new_p[1],q_new_p[2],q_new_p[3],q_new_p[4]));

		


		
		// check continuous collision between q_new and the closest point on RRT
		if (!contcollisionCheckYoubot(Vt_tmp,q_new,pos,pos,n_sample))
		{
			
			////////////////////////////////////////////////////////
			if (!(graph == NULL))
			{
				//std::cout << "debug" << " " << tmp_sz1 << std::endl;
				//tmp = graph;
				
				int **tmp = graph; // temporary place holder for previous graph
				
				// initialize new graph with expanded size
				graph = new int*[Vt.size()+1] ();
				
				for (int i1 = 0; i1 < Vt.size()+1; i1++){
					graph[i1] = new int[Vt.size()+1] ();
				}
				
				// copy tmp -> graph
				for (int z1 = 0; z1 < grh_sz; z1++){
					for (int z2 = 0; z2 < grh_sz; z2++){
						//std::cout << "debug" << std::endl;
						graph[z1][z2] = tmp[z1][z2];
					}
				}
				
				// delete tmp graph
				for (int z3 = 0;z3 < grh_sz; ++z3){
					delete[] tmp[z3];
				}			
				delete[] tmp;
			}
			// if the graph is NULL no need to deal with tmp graph
			else
			{
				graph = new int*[Vt.size()+1] ();
				for (int i1 = 0; i1 < Vt.size()+1; ++i1){
					graph[i1] = new int[Vt.size()+1] ();
				}				
			}
			////////////////////////////////////////////////////////

			
			
			graph[min_idx][Vt.size()] = 1;
			grh_sz = Vt.size()+1;
			Vt.push_back(Points5D(q_new[0]._q1,q_new[0]._q2,q_new[0]._q3,q_new[0]._q4,q_new[0]._q5));

			// check if q_new is close enough to q1 (less than step_size)
			if ((std::abs(atan2(sin(q1[0]._q1- q_new[0]._q1), cos(q1[0]._q1- q_new[0]._q1)))< step_size)&&
			(std::abs(atan2(sin(q1[0]._q2- q_new[0]._q2), cos(q1[0]._q2- q_new[0]._q2)))< step_size)&&
			(std::abs(atan2(sin(q1[0]._q3- q_new[0]._q3), cos(q1[0]._q3- q_new[0]._q3)))< step_size)&&
			(std::abs(atan2(sin(q1[0]._q4- q_new[0]._q4), cos(q1[0]._q4- q_new[0]._q4)))< step_size)&&
			(std::abs(atan2(sin(q1[0]._q5- q_new[0]._q5), cos(q1[0]._q5- q_new[0]._q5)))< step_size)&&
			!contcollisionCheckYoubot(q_new,q1,pos,pos,n_sample))
			{
				
				// same deal as before: graph -> tmp, 
				// expand graph
				// tmp -> expaned graph
				// delete tmp
				int **tmp1 = graph;
				graph = new int*[Vt.size()+1] ();
				for (int i1 = 0; i1 < Vt.size()+1; i1++){
					graph[i1] = new int[Vt.size()+1] ();
				}
				for (int z1 = 0; z1 < grh_sz; z1++){
					for (int z2 = 0; z2 < grh_sz; z2++){
						graph[z1][z2] = tmp1[z1][z2];
					}
				}
				for (int z3 = 0;z3 < grh_sz; z3++){
					delete[] tmp1[z3];
				}			
				delete[] tmp1;


				
				graph[Vt.size()-1][Vt.size()] = 1;
				grh_sz = Vt.size()+1;
				Vt.push_back(Points5D(q1[0]._q1,q1[0]._q2,q1[0]._q3,q1[0]._q4,q1[0]._q5));
				true_exit = true;
			}
		}
		q_new.clear();
		q_rand.clear();
		Vt_tmp.clear();		
		i+=1;
	}

	// flag is set true if collision free
	bool collision_free = true;
	double set_q_new[5];
	
	for (size_t j =0 ; j < Vt.size() ; j++)
	{
		set_q_new[0] = Vt[j]._q1;
		set_q_new[1] = Vt[j]._q2;
		set_q_new[2] = Vt[j]._q3;
		set_q_new[3] = Vt[j]._q4;
		set_q_new[4] = Vt[j]._q5;
		
		collision_free *= (!collisionCheckYoubot(pos, set_q_new));		
	}
	
	if (collision_free==true)
	{
		std::cout << "a collision-free path was found in RRT with " << i << " samples!" << std::endl;
	}

	std::cout << "finding the shortest path" << std::endl;

	// use Dijkstra's algorithm to find the shortest path on RRT
	std::vector<int> path=dijkstra(graph, 0, Vt.size()-1,Vt.size());
	std::vector<Points5D> Vt_new;
	
	for (size_t i = 0 ; i < path.size(); i++){
		Vt_new.push_back(Points5D(Vt[path[i]]._q1, Vt[path[i]]._q2, Vt[path[i]]._q3, Vt[path[i]]._q4, Vt[path[i]]._q5));
	}
	
	for (int z3 = 0;z3 < grh_sz; z3++){
		delete[] graph[z3];
	}			
	delete[] graph;	
		
	return Vt_new;	

}

void motionPlanning::getYoubotPose()
{
	
	listener.waitForTransform(odom, base_footprint,ros::Time::now(), ros::Duration(3.0));

	// transform youbot's origin pose from "base_foorprint" to "odom" coordinate
	listener.transformPose(odom, yb_origin, tf_yb_origin); 
	
	// assign to variables for conveniences
	tf_yb_origin_x = tf_yb_origin.getOrigin().x();
	tf_yb_origin_y = tf_yb_origin.getOrigin().y();			
	tf_yb_origin_yaw = tf::getYaw(tf_yb_origin.getRotation()); 
}

void motionPlanning::callbackJoint (const sensor_msgs::JointState::ConstPtr& state)
{
	start_pt.positions = state->position;
	got_currentposition = true;

}

// This function sets pose, control points, and transformation of youbot
void motionPlanning::setYoubot()
{
	// define the pose of the origin
	yb_origin.frame_id_ = base_footprint;
	yb_origin.stamp_ = ros::Time(0);
	yb_origin.setData(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0.0)));

}

// check collisions between youbots' arms and the obstacles
bool motionPlanning::collisionCheckYoubot(double pos[], double th_r[])
{
	double th[] = {0,th_r[0],th_r[1]-PI/2,th_r[2],th_r[3],PI/2,th_r[4]+PI/2};
	double a[] = {0.167, 0.033, 0.155, 0.135, 0.1136, 0, 0};
	double ap[] = {PI, PI/2, 0, 0, -PI/2, PI/2, 0};
	double d[] = {0.245, 0, 0, 0, 0, 0, 0.05716};

	double yb_org_x = pos[0];
	double yb_org_y = pos[1];
	double yb_org_yaw = pos[2];

	std::vector<fcl::Matrix3f> rot;
	std::vector<fcl::Vec3f> tr;
	std::vector<fcl::Vec3f> ee;
	fcl::Vec3f k(0,0,0);

	std::vector<fcl::Transform3f> A;
	std::vector<fcl::Transform3f> T;

	// mobile base
	std::vector<fcl::Matrix3f> rot_base;
	std::vector<fcl::Vec3f> tr_base;	
	std::vector<fcl::Transform3f> A_base;
	
	rot_base.push_back(fcl::Matrix3f(cos(yb_org_yaw),-sin(yb_org_yaw),0,
	sin(yb_org_yaw), cos(yb_org_yaw),0,
	0,0,1));	
	tr_base.push_back(fcl::Vec3f(yb_org_x,yb_org_y,0));
	A_base.push_back(fcl::Transform3f(rot_base[0],tr_base[0]));
	for (size_t i=0; i < 7; ++i)
	{
		rot.push_back(fcl::Matrix3f(cos(th[i]),-sin(th[i])*cos(ap[i]),sin(th[i])*sin(ap[i]),
		sin(th[i]), cos(th[i])*cos(ap[i]),-cos(th[i])*sin(ap[i]),
		0,sin(ap[i]),cos(ap[i])));

		tr.push_back(fcl::Vec3f(a[i]*cos(th[i]),a[i]*sin(th[i]),d[i]));

		A.push_back(fcl::Transform3f(rot[i],tr[i]));
	}
	for (size_t i=0; i != A.size(); ++i)
	{
		if (i > 0){
			T.push_back(T[i-1]*A[i]);
		}
		else{
			T.push_back(A_base[0]*A[i]);
		}
		ee.push_back(T[i].transform(k));
	}
	// base1
	std::vector<fcl::Matrix3f> rot_base1_tmp;
	std::vector<fcl::Vec3f> tr_base1_tmp;
	fcl::Transform3f A_base1_tmp;
	A_base1_tmp.setIdentity();
	A_base1_tmp.setTranslation(fcl::Vec3f(0,0.0082+0.0025,0));
	
	boost::shared_ptr<fcl::Box> yb_base(new fcl::Box(0.6,0.4,0.124));
	boost::shared_ptr<fcl::Box> yb_arm0(new fcl::Box(0.22,0.2,0.09));
	boost::shared_ptr<fcl::Box> yb_arm1(new fcl::Box(0.2,0.2,0.12));
	boost::shared_ptr<fcl::Box> yb_arm2(new fcl::Box(0.22,0.08,0.0823));
	boost::shared_ptr<fcl::Box> yb_arm3(new fcl::Box(0.06,0.2,0.075));
	boost::shared_ptr<fcl::Box> yb_arm4(new fcl::Box(0.06,0.2524,0.1));
	
	fcl::Transform3f D_tf_yb_base, D_tf_yb_arm0, D_tf_yb_arm1,  D_tf_yb_arm2,  D_tf_yb_arm3,  D_tf_yb_arm4;
	D_tf_yb_base.setIdentity();
	D_tf_yb_arm0.setIdentity();
	D_tf_yb_arm1.setIdentity();
	D_tf_yb_arm2.setIdentity();
	D_tf_yb_arm3.setIdentity();
	D_tf_yb_arm4.setIdentity();
	D_tf_yb_base.setTranslation(fcl::Vec3f(0, 0, 0.022));
	D_tf_yb_arm0.setTranslation(fcl::Vec3f(-0.11, 0, -0.005));
	D_tf_yb_arm1.setTranslation(fcl::Vec3f(0, 0, 0.02));
	D_tf_yb_arm2.setTranslation(fcl::Vec3f(0.0740, 0, -0.0411));
	D_tf_yb_arm3.setTranslation(fcl::Vec3f(0, 0.07, 0.0375));
	D_tf_yb_arm4.setTranslation(fcl::Vec3f(0, 0.09346, 0));
	
	fcl::Transform3f TI_yb_base, TI_yb_arm0, TI_yb_arm1,  TI_yb_arm2,  TI_yb_arm3,  TI_yb_arm4;
	TI_yb_base.setIdentity();
	TI_yb_base.setTranslation(fcl::Vec3f(0, 0, 0.084));
	TI_yb_arm0.setIdentity();
	TI_yb_arm0.setTranslation(fcl::Vec3f(0.143,0,0.046+0.084));
	TI_yb_arm1.setIdentity();
	TI_yb_arm1.setRotation(fcl::Matrix3f(cos(th[1]),-sin(th[1]),0,sin(th[1]), cos(th[1]),0, 0,0,1));	
	TI_yb_arm2.setIdentity();
	TI_yb_arm2.setRotation(fcl::Matrix3f(cos(th[2]),-sin(th[2]),0,sin(th[2]), cos(th[2]),0, 0,0,1));	
	TI_yb_arm3.setIdentity();
	TI_yb_arm3.setRotation(fcl::Matrix3f(cos(th[3]-PI/2),-sin(th[3]-PI/2),0,sin(th[3]-PI/2), cos(th[3]-PI/2),0, 0,0,1));	
	TI_yb_arm4.setIdentity();	
	TI_yb_arm4.setRotation(fcl::Matrix3f(cos(th[4]-PI/2),-sin(th[4]-PI/2),0,sin(th[4]-PI/2), cos(th[4]-PI/2),0, 0,0,1));	
	
	fcl::CollisionObject co_yb_base(yb_base, A_base[0]*TI_yb_base*D_tf_yb_base);
	fcl::CollisionObject co_yb_arm0(yb_arm0, A_base[0]*TI_yb_arm0*D_tf_yb_arm0);
	fcl::CollisionObject co_yb_arm1(yb_arm1, T[0]*TI_yb_arm1*D_tf_yb_arm1);
	fcl::CollisionObject co_yb_arm2(yb_arm2, T[1]*TI_yb_arm2*D_tf_yb_arm2);
	fcl::CollisionObject co_yb_arm3(yb_arm3, T[2]*TI_yb_arm3*D_tf_yb_arm3);
	fcl::CollisionObject co_yb_arm4(yb_arm4, T[3]*TI_yb_arm4*D_tf_yb_arm4);

	// create obstacle objects
	boost::shared_ptr<fcl::Box> obs_m1(new fcl::Box(0.4,0.4,0.2));
	boost::shared_ptr<fcl::Box> obs_m2(new fcl::Box(0.05,0.05,0.15));
	boost::shared_ptr<fcl::Box> obs_m3(new fcl::Box(0.05,0.05,0.15));
	boost::shared_ptr<fcl::Box> obs_m4(new fcl::Box(0.05,0.25,0.05));

	fcl::Transform3f tr_obs_m1, tr_obs_m2, tr_obs_m3, tr_obs_m4;
	tr_obs_m1.setIdentity();
	tr_obs_m1.setTranslation(fcl::Vec3f(0.6, 0, 0.1));	
	tr_obs_m2.setIdentity();
	tr_obs_m2.setTranslation(fcl::Vec3f(0.5, 0.1, 0.275));	
	tr_obs_m3.setIdentity();
	tr_obs_m3.setTranslation(fcl::Vec3f(0.5, -0.1, 0.275));		
	tr_obs_m4.setIdentity();
	tr_obs_m4.setTranslation(fcl::Vec3f(0.5, 0, 0.375));		
	
	fcl::CollisionObject co_obs_m1(obs_m1, tr_obs_m1);
	fcl::CollisionObject co_obs_m2(obs_m2, tr_obs_m2);
	fcl::CollisionObject co_obs_m3(obs_m3, tr_obs_m3);
	fcl::CollisionObject co_obs_m4(obs_m4, tr_obs_m4);
	
	fcl::CollisionResult col_result;
	fcl::CollisionResult col_result11, col_result21, col_result31, col_result41, col_result51, col_result61;
	fcl::CollisionResult col_result12, col_result22, col_result32, col_result42, col_result52, col_result62;
	fcl::CollisionResult col_result13, col_result23, col_result33, col_result43, col_result53, col_result63;
	fcl::CollisionResult col_result14, col_result24, col_result34, col_result44, col_result54, col_result64;
	fcl::CollisionResult col_result_arm_2_4;
	
	fcl::CollisionRequest col_request;
	
	bool record = true;
	
	fcl::collide(&co_yb_base, &co_obs_m1, col_request, col_result11);
	record*=!col_result11.isCollision();
	fcl::collide(&co_yb_arm0, &co_obs_m1, col_request, col_result21);
	record*=!col_result21.isCollision();
	fcl::collide(&co_yb_arm1, &co_obs_m1, col_request, col_result31);
	record*=!col_result31.isCollision();
	fcl::collide(&co_yb_arm2, &co_obs_m1, col_request, col_result41);
	record*=!col_result41.isCollision();
	fcl::collide(&co_yb_arm3, &co_obs_m1, col_request, col_result51);
	record*=!col_result51.isCollision();
	fcl::collide(&co_yb_arm4, &co_obs_m1, col_request, col_result61);
	record*=!col_result61.isCollision();

	fcl::collide(&co_yb_base, &co_obs_m2, col_request, col_result12);
	record*=!col_result12.isCollision();
	fcl::collide(&co_yb_arm0, &co_obs_m2, col_request, col_result22);
	record*=!col_result22.isCollision();
	fcl::collide(&co_yb_arm1, &co_obs_m2, col_request, col_result32);
	record*=!col_result32.isCollision();
	fcl::collide(&co_yb_arm2, &co_obs_m2, col_request, col_result42);
	record*=!col_result42.isCollision();
	fcl::collide(&co_yb_arm3, &co_obs_m2, col_request, col_result52);
	record*=!col_result52.isCollision();
	fcl::collide(&co_yb_arm4, &co_obs_m2, col_request, col_result62);
	record*=!col_result62.isCollision();
	
	fcl::collide(&co_yb_base, &co_obs_m3, col_request, col_result13);
	record*=!col_result13.isCollision();
	fcl::collide(&co_yb_arm0, &co_obs_m3, col_request, col_result23);
	record*=!col_result23.isCollision();
	fcl::collide(&co_yb_arm1, &co_obs_m3, col_request, col_result33);
	record*=!col_result33.isCollision();
	fcl::collide(&co_yb_arm2, &co_obs_m3, col_request, col_result43);
	record*=!col_result43.isCollision();
	fcl::collide(&co_yb_arm3, &co_obs_m3, col_request, col_result53);
	record*=!col_result53.isCollision();
	fcl::collide(&co_yb_arm4, &co_obs_m3, col_request, col_result63);
	record*=!col_result63.isCollision();

	fcl::collide(&co_yb_base, &co_obs_m4, col_request, col_result14);
	record*=!col_result14.isCollision();
	fcl::collide(&co_yb_arm0, &co_obs_m4, col_request, col_result24);
	record*=!col_result24.isCollision();
	fcl::collide(&co_yb_arm1, &co_obs_m4, col_request, col_result34);
	record*=!col_result34.isCollision();
	fcl::collide(&co_yb_arm2, &co_obs_m4, col_request, col_result44);
	record*=!col_result44.isCollision();
	fcl::collide(&co_yb_arm3, &co_obs_m4, col_request, col_result54);
	record*=!col_result54.isCollision();
	fcl::collide(&co_yb_arm4, &co_obs_m4, col_request, col_result64);
	record*=!col_result64.isCollision();

//  arm 2 and arm 4
	fcl::collide(&co_yb_arm2, &co_yb_arm4, col_request, col_result_arm_2_4);
	record*=!col_result_arm_2_4.isCollision();
	return !record;	
}

// check continuous collisions between youbots' arms and the obstacles
bool motionPlanning::contcollisionCheckYoubot(std::vector<Points5D> pnt_q1,std::vector<Points5D> pnt_q2, double pos1[], double pos2[], double n_sample)
{
	double q1[5], q2[5];
	q1[0] = pnt_q1[0]._q1;
	q1[1] = pnt_q1[0]._q2;
	q1[2] = pnt_q1[0]._q3;
	q1[3] = pnt_q1[0]._q4;
	q1[4] = pnt_q1[0]._q5;	
	q2[0] = pnt_q2[0]._q1;
	q2[1] = pnt_q2[0]._q2;
	q2[2] = pnt_q2[0]._q3;
	q2[3] = pnt_q2[0]._q4;
	q2[4] = pnt_q2[0]._q5;		
	bool comp_val = true;
	double th_r[5];
	double pos_r[3];
	for (size_t i=0; i < n_sample+1; i++)
	{
		
		for (size_t j=0; j < 5; j++)
		{
			th_r[j]=q1[j]+i*(q2[j]-q1[j])/n_sample;
		}
		for (size_t k=0; k < 3; k++)
		{
			pos_r[k]=pos1[k]+i*(pos2[k]-pos1[k])/n_sample;
		}
		
		comp_val*=!collisionCheckYoubot(pos_r, th_r);
	}	
	
	
	return !comp_val;
}

// generate way points and generate PID control effort
geometry_msgs::Twist motionPlanning::wayPoint(std::vector<Points3D> wp, unsigned int cnt, double kp, double ki, double kd, 
double kp_t, double ki_t, double kd_t)
{
	geometry_msgs::Twist vel;
	std::vector<Points3D> robot_pos;
	std::vector<Points3D> goal_pos;
	robot_pos.push_back(Points3D(tf_yb_origin_x, tf_yb_origin_y, tf_yb_origin_yaw));
	goal_pos.push_back(Points3D(wp[cnt]._x, wp[cnt]._y, wp[cnt]._z));
	double dist_robot_goal = distPointPoint(robot_pos, goal_pos);
	double thres = 0.01; 
	double err_prev_x = err_x;
	double err_prev_y = err_y;
	double err_prev_th = err_th;	
	double tmp_th;
	err_x = (wp[cnt]._x - tf_yb_origin_x);
	err_y = (wp[cnt]._y - tf_yb_origin_y);
	
	err_th = atan2(sin(wp[cnt]._z - tf_yb_origin_yaw),cos(wp[cnt]._z - tf_yb_origin_yaw));
	
	err_acum_x += err_x;
	err_acum_y += err_y;
	err_acum_th += err_th;
	double err_def_x = err_x - err_prev_x;
	double err_def_y = err_y - err_prev_y;
	double err_def_th = err_th - err_prev_th;
	
	// PID control
	if (dist_robot_goal < thres)
	{
		vel.linear.x = 0;
		vel.linear.y = 0;
		vel.angular.z = 0;
	}
	else
	{

		vel.linear.x = kp * err_x + ki * err_acum_x + kd * err_def_x;
		vel.linear.y = kp * err_y + ki * err_acum_y + kd * err_def_y;
		vel.angular.z = kp_t * err_th + ki_t * err_acum_th + kd_t * err_def_th;;	
	}	
	return vel;
}

// Euclidean distance between two points in R^3
double motionPlanning::distPointPoint(std::vector<Points3D> v1, std::vector<Points3D> v2)
{
	return sqrt(pow(v1[0]._x-v2[0]._x,2) + pow(v1[0]._y-v2[0]._y,2)
	+ pow(v1[0]._z-v2[0]._z,2));
}

// dot product v1 dot v2
double motionPlanning::dotP(std::vector<Points3D> v1, std::vector<Points3D> v2)
{
	return v1[0]._x * v2[0]._x + v1[0]._y * v2[0]._y + v1[0]._z * v2[0]._z;
}

int main(int argc, char **argv)
{
	// assign proper names for robot base, command velocity, and odom topic
	base_footprint = "base_footprint";		// a name for robot's coordinate
	odom = "odom";							// a name for world's coordinate
	cmd_vel = "cmd_vel";					
	srand( time(0));

	// initialize as a ROS program
    ros::init(argc, argv,"sample2_node",ros::init_options::NoSigintHandler);  
    
    // create node handle
    ros::NodeHandle n;	
    
    // create motionPlanning object with the node handler
    motionPlanning mp(n);
    
	return 0;
}
