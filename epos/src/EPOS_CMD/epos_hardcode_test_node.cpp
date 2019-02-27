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

#define COUNTS_PER_REV 128
//6.5 rev of output ~= to 2k rev motor

std::vector<int> motorIDs;
std::vector<long> vels;

/**void motorCommandCallback(const epos::wheel_drive &msg)
{
//std::cout << msg.numberItems;
	for (int i = 0; i < msg.numberItems; ++i)
	{

			motorIDs[i] = msg.motorIDs[i];
			vels[i] = msg.velocities[i];
	}
}*/


int main(int argc, char** argv)
{
		ros::init(argc,argv,"subscriber");
		ros::NodeHandle nh;
		//ros::Subscriber sub = nh.subscribe("/velocities", 10, motorCommandCallback);
		//ros::NodeHandle n;
		//ros::Publisher pub = n.advertise<std_msgs::Int64MultiArray>("/motor_pos", 1000);


		motorIDs.push_back(1);
		//motorIDs.push_back(2);
    //motorIDs.push_back(3);
    //motorIDs.push_back(4);
    //std::vector<unsigned short> motorIDshort;
		//motorIDshort.push_back(1);
		//motorIDshort.push_back(2);
		//motorIDs.push_back(3);
		//motorIDs.push_back(4);

    vels.push_back(7000);
    //vels.push_back(0);
    //vels.push_back(0);
    std::vector<long> stopVels;
    stopVels.push_back(0);

		int baudrate = 1000000;
    //std::cout << motorIDs[0] << "__" << motorIDs[1] << std::endl;

		std::vector<int> positions;


		epos_cmd motorController(motorIDs, baudrate);

		if (motorController.openDevices())
		{
				ROS_FATAL("Motor not opened");
				return 1;
		}
		try{
        float iteration = 0;
        int check = motorController.setMode(motorIDs, epos_cmd::OMD_PROFILE_VELOCITY_MODE);

				ros::Rate rate(20);
        int num_counts =0;
				while(ros::ok() && num_counts < 2000*COUNTS_PER_REV)// && iteration < 500)
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
                  /**geometry_msgs::Twist command = *(ros::topic::waitForMessage<geometry_msgs::Twist>("/cmd_vel"));
                  long Vl = (command.linear.x + 0.5*command.angular.z)*400;
                  long Vr = -(command.linear.x + 0.5*command.angular.z)*400;

                  std::cout << Vl << "__" << Vr << std::endl;

                  vels[0] = Vl;
                  vels[1] = Vl;
                  vels[2] = Vr;
                  vels[3] = Vr;*/

                ROS_WARN("GOOD");
                motorController.goToVel(motorIDs, vels);
            }

						motorController.getPosition(motorIDs, positions);

						//std_msgs::Int64MultiArray motorPos;
						for (int i = 0; i < positions.size();++i)
						{
							std::cout << "Position " << i << " is " << positions[i] << std::endl;
						}
            num_counts = positions[0];
            positions.clear();
						//pub.publish(motorPos);*/

            //++iteration;
						rate.sleep();
						//ros::spinOnce();
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
