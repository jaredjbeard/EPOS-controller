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
		motorIDs.push_back(2);
		//motorIDs.push_back(3);
		//motorIDs.push_back(4);
    std::vector<long> vels;
    vels.push_back(300);
    vels.push_back(300);
    std::vector<long> stopVels;
    stopVels.push_back(0);
    stopVels.push_back(0);
		int baudrate = 1000000;

		epos_cmd motorController(motorIDs, baudrate);

		if (motorController.openDevices())
		{
				ROS_FATAL("Motor not opened");
				return 1;
		}
		try{
        float iteration = 0;
				while(ros::ok() && iteration < 500)
				{
            if (motorController.setMode(motorIDs, epos_cmd::OMD_PROFILE_VELOCITY_MODE) == 0 && motorController.prepareMotors(motorIDs) == 0)
            {
                //ROS_INFO("GOTO");
                motorController.goToVel(motorIDs, vels);
            }
            ++iteration;
				}
        motorController.goToVel(motorIDs,stopVels);
        motorController.closeDevices();
				return 0;
		} catch (const std::exception& e)
		{
      motorController.closeDevices();
		}
}
