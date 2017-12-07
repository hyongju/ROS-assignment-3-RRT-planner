// A sample C++ code for assignment #3 - Part II
// This is a simple motion planning program to move robot arms using RRT

#include "motionPlanning.h"



int main(int argc, char **argv)
{				
	srand( time(0));

	// initialize as a ROS program
	ros::init(argc, argv,"rrt_youbot_node",ros::init_options::NoSigintHandler);  

	// create node handle
	ros::NodeHandle n;	

	// create motionPlanning object with the node handler
	motionPlanning mp(n);

	return 0;
}
