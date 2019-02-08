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
#include <geometry_msgs/Twist.h>



#include <stdio.h>

int main(int argc, char** argv)
{
		ros::init(argc,argv,"subscriber");
		ros::NodeHandle nh;

		std::vector<int> motorIDs;
		motorIDs.push_back(1);
		motorIDs.push_back(2);
    motorIDs.push_back(3);
    motorIDs.push_back(4);
    //std::vector<unsigned short> motorIDshort;
		//motorIDshort.push_back(1);
		//motorIDshort.push_back(2);
		//motorIDs.push_back(3);
		//motorIDs.push_back(4);
    std::vector<long> vels;
    vels.push_back(0);
    vels.push_back(0);
    vels.push_back(0);
    vels.push_back(0);
    std::vector<long> stopVels;
    stopVels.push_back(0);
    stopVels.push_back(0);
		int baudrate = 1000000;
    std::cout << motorIDs[0] << "__" << motorIDs[1] << std::endl;
		epos_cmd motorController(motorIDs, baudrate);
    //std::vector<int> motors;
    //std::vector<long> velocities;

		if (motorController.openDevices())
		{
				ROS_FATAL("Motor not opened");
				return 1;
		}
		try{
        float iteration = 0;
        int check = motorController.setMode(motorIDs, epos_cmd::OMD_PROFILE_VELOCITY_MODE);
				while(ros::ok() && iteration < 500)
				{
            if ( check == 0 && motorController.prepareMotors(motorIDs) == 0)
            {
                //ROS_INFO("GOTO");
                  /**epos::wheel_drive command = *(ros::topic::waitForMessage<epos::wheel_drive>("/drive"));
                  for (int i = 0; i < motorIDs.size(); ++ i)
                  {
                    motors.push_back(command.motorIDs[i]);
                    velocities.push_back(command.velocities[i]);
                  }*/
                  geometry_msgs::Twist command = *(ros::topic::waitForMessage<geometry_msgs::Twist>("/cmd_vel"));
                  long Vl = command.linear.x + 0.5*command.angular.z;
                  long Vr = command.linear.x - 0.5*command.angular.z;
                  Vl*=400;
                  Vr*=400;

                  std::cout << Vl << "__" << Vr << std::endl;

                  vels[0] = Vl;
                  vels[1] = Vr;
                  vels[2] = Vl;
                  vels[3] = Vr;


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
