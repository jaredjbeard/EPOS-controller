#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <epos/wheel_drive.h>





double lVel = 0;
double rVel = 0;
double normal;
double maxV = 7000;


void cmdVelCallBack(const geometry_msgs::Twist& msg)
{


	//std::cout<<"Linear"<<std::endl;
	//std::cout<<msg.linear.x<<std::endl;
	//std::cout<<"Angular"<<std::endl;
	//std::cout<<msg.angular.z<<std::endl;

	rVel = (msg.linear.x + msg.angular.z); //Unnormalized right and left values
	lVel = (msg.linear.x - msg.angular.z);

	//std::cout<<"RIGHT"<<std::endl;
	//std::cout<<rVel<<std::endl;

	//std::cout<<"LEFT"<<std::endl;
	//std::cout<<lVel<<std::endl;



	normal = std::max(rVel,lVel);
	//std::cout<<"Normalizer"<<std::endl;
	//std::cout<<normal<<std::endl;

	if (normal > 1)
	{
		rVel = (rVel/normal);
		lVel = (lVel/normal);
	}

	rVel = -rVel * maxV;
	lVel = lVel * maxV;


	//std::cout<<"RIGHT"<<std::endl;
	//std::cout<<rVel<<std::endl;

	//std::cout<<"LEFT"<<std::endl;
	//std::cout<<lVel<<std::endl;



	//std::cout<<""<<std::endl;


}

int main(int argc, char** argv)
{


	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/cmd_vel",10,&cmdVelCallBack);



	//ros::init(argc, argv, "talker");
	//ros::NodeHandle n;
	ros::Publisher chatter_pub = nh.advertise<epos::wheel_drive>("/velocities", 1000);
	ros::Rate rate(20);
	while(ros::ok())
	{
	epos::wheel_drive msg;
	msg.motorIDs = {1,2,3,4};
	//msg.driveMode = rVel;
	//std::cout << msg.velocities << std::endl;
	//std::cout << lVel << "__" << rVel << std::endl;
	msg.command = {rVel, lVel, lVel, rVel};
	msg.numberItems = 4;
	chatter_pub.publish(msg);

	rate.sleep();
	ros::spinOnce();
	}
}
