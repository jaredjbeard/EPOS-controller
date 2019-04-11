/*
   Maxon Motor Controller Finite State Machine node epos_test_node
   epos_FSM_node.cpp
   Purpose: Deployable EPOS code for most projects un epos commands and test behavior

   @author Jared Beard
   @version 1.0 3/4/19
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <epos/epos_cmd.h>
#include <epos/wheel_drive.h>
#include <std_msgs/Int64MultiArray.h>

#include <stdio.h>
#include <cmath>
#include <string>

std::vector<int> motorIDs;
std::vector<long> cmd;
std::string operationMode; //1 = velocity, 2 = torque, 3 = current, 4 = position, 5 = homing

void motorCommandCallback(const epos::wheel_drive &msg)
{
		for (int i = 0; i < msg.numberItems; ++i)
		{
				motorIDs[msg.motorIDs[i]-1] = msg.motorIDs[i];
				cmd[msg.motorIDs[i]-1] = msg.command[i];
		}
		operationMode = msg.driveMode;
}


int main(int argc, char** argv)
{
		// ROS INITILIZATION ------------------------------------------------------------------------
		ros::init(argc,argv,"subscriber");
		ros::NodeHandle nh;
		ros::Subscriber sub = nh.subscribe("/epos_cmd", 10, motorCommandCallback);
		ros::NodeHandle n;
		ros::Publisher pub = n.advertise<std_msgs::Int64MultiArray>("/motor_positions", 1000);

		// IMPORT CONFIGURATION ---------------------------------------------------------------------
		nh.param("motors_ids", motorIDs, {1,2,3,4});
		int baudrate;
		nh.param("baudrate", baudrate, 1000000);

		for (int i = 0; i < motorIDs.size(); ++i) cmd.push_back(0);

		double gearRatio, countsPerRev;
		nh.param("gear_ratio", gearRatio, 74.0);
		nh.param("counts_per_rev", countsPerRev, 128.0);

		double countsPreRevShaft = countsPerRev*gearRatio;

		// EPOS CONFIGURATION/PREPARATION------------------------------------------------------------
		epos_cmd motorController(motorIDs, baudrate);
		if (!motorController.openDevices())
		{
				ROS_FATAL("Motor not opened");
				return 1;
		}

		motorController.prepareMotors(motorIDs);
		// FSM --------------------------------------------------------------------------------------
		bool runMotors = true;
		ros::Rate rate(20);
		std::vector<int> positions;
		while(ros::ok() && runMotors)
		{
				if(motorController.prepareMotors(motorIDs))
				{
						// VELOCITY -----------------------------------------------------------------------------
						if (operationMode == "velocity")
						{
								motorController.goToVel(motorIDs, cmd);

								// VELOCITY -----------------------------------------------------------------------------
						} else if (operationMode == "torque")
						{
								motorController.goToTorque(motorIDs, cmd, gearRatio);

								// CURRENT ------------------------------------------------------------------------------
						} else if (operationMode == "current")
						{

								// POSITION -----------------------------------------------------------------------------
						} else if (operationMode == "position")
						{

								// HOMING -------------------------------------------------------------------------------
						} else if (operationMode == "homing")
						{

								// FAILED -------------------------------------------------------------------------------
						} else
						{
								ROS_WARN("Invalid/missing operation mode");
						}

						motorController.getPosition(motorIDs, positions);
						std_msgs::Int64MultiArray motorPos;
						for (int i = 0; i < positions.size(); ++i) motorPos.data[i] = positions[i];
						//motorPosPrev.data[i] = motorPos.data[i];
						pub.publish(motorPos);
				}

				rate.sleep();
				ros::spinOnce();
		}

		/**std_msgs::Int64MultiArray motorPosPrev;

		   for (int i = 0; i < motorIDs.size(); ++i) motorPosPrev.data.push_back(0);
		   for (int i = 0; i < motorIDs.size(); ++i) motorPosPrev.data.push_back(1);

		   while(ros::ok())     // && iteration < 500)
		   {
		    // I suspect these changes may need to be made on a per motor basis to get desired behavior
		    bool moving = false;
		    bool fault = false;
		    for (int i = 0; i < vels.size(); ++i) if (vels[i] != 0) moving = true;
		    for (int i = 0; i < motorIDs.size(); ++i) if (motorPosPrev.data[i] == motorPos.data[i]) fault = true;
		    if (!( moving && !fault))
		    {

		        ROS_INFO("FAULT TRIGGERED");
		        prepareCheck = motorController.prepareMotors(motorIDs);

		    }


		    //++iteration;
		   }		 */

		// STOP AND CLOSE MOTORS---------------------------------------------------------------------
		std::vector<long> stopVels;
		for (int i = 0; motorIDs.size(); ++i) stopVels.push_back(0);
		motorController.goToVel(motorIDs,stopVels);
		motorController.closeDevices();
		return 0;

}
