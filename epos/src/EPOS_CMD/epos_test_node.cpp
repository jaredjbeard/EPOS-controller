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


#include <stdio.h>

int main(int argc, char** argv)
{
		ros::init(argc,argv,"subscriber");
		ros::NodeHandle nh;

		std::vector<int> motorIDs;
		motorIDs.push_back(1);
		//motorIDs.push_back(2);
		//motorIDs.push_back(3);
		//motorIDs.push_back(4);
		int baudrate = 1000000;

		epos_cmd motorController(motorIDs, baudrate);

		if (motorController.OpenDevices())
		{
				std::cout << "Motor not opened" << std::endl;
				return 1;
		}
		try{
				while(ros::ok())
				{

				}

				return 0;
		} catch (const std::exception& e)
		{
      motorController.CloseDevices();
		}
}
