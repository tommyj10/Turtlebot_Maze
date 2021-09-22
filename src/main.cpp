#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cstring>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "TurtleController.h"
#include "VelocityController.h"
#include "LaserScanner.h"
#include "IMU.h"

/**
* This program implements a wall following turtlebot maze solver. Four high level classes are used to control the robot
* and receive feedback-signals: 
*
* TurtleController:			Navigation and movement logic. Performs the heavy lifting.
* VelocityController:		Publishes messages to the cmd_vel topic 
* LaserScanner:				Keeps a record of laser scan ranges 
* IMU:						Keeps a record of IMU sensor readings
*
*/


bool buttonPushed = false;

void buttonPushedCallback(const diagnostic_msgs::DiagnosticArray &msg){
	std::cout << "************************************************** " << std::endl;
	//std::cout << msg.status.size() << std::endl;

	if (std::strcmp(msg.status.at(4).message.c_str(), "Pushed Nothing")){
		buttonPushed = true;
		std::cout << "BUTTON PUSHED " << std::endl;
	}

	std::cout << "************************************************** " << std::endl;
}

int main(int argc, char **argv){

    // create a ROS node
    ros::init(argc, argv, "WallFollower");
    ros::NodeHandle nh;

	// Initialise a publisher that talks to the turtlebots movement controller
	ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	
	VelocityController velcmd(&velocity_publisher);
	LaserScanner laserscan;
	TurtleController turtlecontroller(&laserscan, &velcmd);

	ros::Subscriber laser_subscriber = nh.subscribe("/scan", 1000, &LaserScanner::laserCallback, &laserscan);

	ros::Rate loop_rate(10);

	std::cout << "Waiting for laser... ";
	while (!laserscan.dataIsAvailable()){ 
		ros::spinOnce(); 
		loop_rate.sleep();
	}
    std::cout << "Done." << std::endl;
    
	// The heavy lifting happens in turtlecontroller.run()
    while (ros::ok())
    { 
        ros::spinOnce();
		turtlecontroller.run();
        loop_rate.sleep();
    }

    return 0;
}













