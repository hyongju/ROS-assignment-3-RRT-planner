#ifndef MOTIONPLANNING_H
#define MOTIONPLANNING_H

#define PI	3.141592


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

#include "tuple.h"


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
		
		// a few empty strings for later use...
		std::string base_footprint;
		std::string cmd_vel;
		std::string odom;
		
		
		       
};



#endif
