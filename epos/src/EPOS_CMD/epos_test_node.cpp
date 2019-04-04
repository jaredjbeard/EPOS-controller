/*
   Maxon Motor Controller test node epos_test_node
   epos_test_node.cpp
   Purpose: Run epos commands and test behavior

   @author Jared Beard
   @version 1.0 1/18/19
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <epos/epos_cmd.h>
#include <epos/wheel_drive.h>
#include <std_msgs/Int64MultiArray.h>
//#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <cmath>

#define COUNTS_PER_REV 128
#define COUNTS_PER_REV_SHAFT (128.0*2000/6.5)
//6.5 rev of output ~= to 2k rev motor

std::vector<int> motorIDs;
std::vector<long> vels;

void motorCommandCallback(const epos::wheel_drive &msg)
{
//std::cout << msg.numberItems;
		for (int i = 0; i < msg.numberItems; ++i)
		{

				motorIDs[i] = msg.motorIDs[i];
				vels[i] = msg.velocities[i];
		}
}


int main(int argc, char** argv)
{
		ros::init(argc,argv,"subscriber");
		ros::NodeHandle nh;
		ros::Subscriber sub = nh.subscribe("/velocities", 10, motorCommandCallback);
		ros::NodeHandle n;
		ros::Publisher pub = n.advertise<std_msgs::Int64MultiArray>("/motor_pos", 1000);


		motorIDs.push_back(1);
		motorIDs.push_back(2);
		motorIDs.push_back(3);
		motorIDs.push_back(4);
		//std::vector<unsigned short> motorIDshort;
		//motorIDshort.push_back(1);
		//motorIDshort.push_back(2);
		//motorIDs.push_back(3);
		//motorIDs.push_back(4);

		vels.push_back(0);
		vels.push_back(0);
		vels.push_back(0);
		vels.push_back(0);
		std_msgs::Int64MultiArray motorPosPrev;
		std_msgs::Int64MultiArray motorPos;
		for (int i = 0; i < motorIDs.size(); ++i)	motorPosPrev.data.push_back(0);
		for (int i = 0; i < motorIDs.size(); ++i)	motorPosPrev.data.push_back(1);
		std::vector<long> stopVels = vels;
		int baudrate = 1000000;
		//std::cout << motorIDs[0] << "__" << motorIDs[1] << std::endl;

		std::vector<int> positions;


		epos_cmd motorController(motorIDs, baudrate);

		if (!motorController.openDevices())
		{
				ROS_FATAL("Motor not opened");
				return 1;
		}
		try{
				float iteration = 0;
				int modeCheck = motorController.setMode(motorIDs, epos_cmd::OMD_PROFILE_VELOCITY_MODE);
				int prepareCheck = motorController.prepareMotors(motorIDs);
				ros::Rate rate(20);
				while(ros::ok()) // && iteration < 500)
				{
						// I suspect these changes may need to be made on a per motor basis to get desired behavior
						bool moving = false;
						bool fault = false;
						for (int i = 0; i < vels.size(); ++i)	if (vels[i] != 0) moving = true;
						for (int i = 0; i < motorIDs.size(); ++i)	if (motorPosPrev.data[i] == motorPos.data[i]) fault = true;
						if (!( moving && modeCheck && !fault))
						{

								ROS_INFO("FAULT TRIGGERED");
								prepareCheck = motorController.prepareMotors(motorIDs);

						}
						prepareCheck = motorController.goToVel(motorIDs, vels);
						motorController.getPosition(motorIDs, positions);

						for (int i = 0; i < positions.size(); ++i)
						{
								motorPosPrev.data[i] = motorPos.data[i];
								motorPos.data[i] = positions[i];
						}

						pub.publish(motorPos);

						//++iteration;
						rate.sleep();
						ros::spinOnce();
				}
				motorController.goToVel(motorIDs,stopVels);
				motorController.closeDevices();
				return 0;
		} catch (const std::exception& e)
		{
				motorController.closeDevices();
				return 1;
		}
}
