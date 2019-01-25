/*
	Maxon Motor Controller motor_cmd_node
	motor_cmd_node.cpp
	Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

	@author Jared Beard
	@version 1.0 11/13/18
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <epos/motor_cmd.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <Definitions.h>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/times.h>
#include <sys/time.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/cmd_vel",10,&cmdReceived);
  ros::Subscriber clearFaultSub = nh.subscribe("/joy",10,&clearFaultCallback);

	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	SetDefaultParameters();

	if((lResult = ParseArguments(argc, argv))!=MMC_SUCCESS)
	{
		return lResult;
	}

	PrintSettings();

			if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("OpenDevice", lResult, ulErrorCode);
				return lResult;
			}

			if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("PrepareDemo", lResult, ulErrorCode);
				return lResult;
			}
/*
			if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("Demo", lResult, ulErrorCode);
				return lResult;
			}
*/
    ros::spin();
            if((lResult = DisableDemo(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("DisableDemo", lResult, ulErrorCode);
				return lResult;
			}
			if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("CloseDevice", lResult, ulErrorCode);
				return lResult;
			}
    return lResult;
}
