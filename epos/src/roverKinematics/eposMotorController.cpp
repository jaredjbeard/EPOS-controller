#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <epos/wheel_drive.h>





double lVel;
double rVel;
double normal;
double maxV = 7000;
double forward = 1;
double i = 0;
double RB;
double prevRB;
double LB;
double prevLB;
double X;
double prevX;
double B;
double prevB;

void joyCallBack(const sensor_msgs::JoyConstPtr& msg)
{

//////////////////////INCREASE SPEED/////////////////
//RB toggle==========================================
	if(msg->buttons[5])//RB button 
	{
		RB = 1;
	}

	if(!msg->buttons[5])//RB button 
	{
		RB = 0;
	}

	if(msg->buttons[5] != prevRB && !prevRB)//RB button 
	{
		i = i + 1;
		///////////////SPPED LIMITERS/////////////////////////
		if(i >= 10)
		{
			i = 10;
		}
		if(i < 0)
		{
			i = 0;
		}

//END SPEED LIMITERS===================================
	}

	prevRB = RB;
//END RB toggle=======================================

//////////////////////DECREASE SPEED/////////////////
//LB toggle==========================================
	if(msg->buttons[4])//LB button 
	{
		LB = 1;
	}

	if(!msg->buttons[4])//LB button 
	{
		LB = 0;
	}

	if(msg->buttons[4] != prevLB && !prevLB)//LB button 
	{
		i = i - 1;

		///////////////SPPED LIMITERS/////////////////////////
		if(i >= 10)
		{
			i = 10;
		}
		if(i < 0)
		{
			i = 0;
		}

//END SPEED LIMITERS===================================
	}

	prevLB = LB;
//END LB toggle=======================================




/////////////////FORWARD/////////////////////////////
//X toggle==========================================
	if(msg->buttons[2])//X button 
	{
		X = 1;
	}

	if(!msg->buttons[2])//X button 
	{
		X = 0;
	}

	if(msg->buttons[2] != prevX && !prevX)//X button 
	{
		forward = 1;
	}

	prevX = X;
//END A toggle=======================================



/////////////REVERSE/////////////////////////////////
//B toggle==========================================
	if(msg->buttons[1])//B button 
	{
		B = 1;
	}

	if(!msg->buttons[1])//LB button 
	{
		B = 0;
	}

	if(msg->buttons[1] != prevB && !prevB)//B button 
	{
		forward = -1;
	}

	prevB = B;
//END B toggle=======================================



	if(forward == 1)
	{
	std::cout<<"FORWARD"<<std::endl;
	}
	else
	{
	std::cout<<"REVERSE"<<std::endl;
	}

	std::cout<<"SPEED:"<<i<<std::endl;	

	

}


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



	normal = std::max(std::abs(rVel),std::abs(lVel));
	//std::cout<<"Normalizer"<<std::endl;
	//std::cout<<normal<<std::endl;
	
	if (normal > 1)
	{
		rVel = (rVel/normal);
		lVel = (lVel/normal);
	}
	
	//std::cout<<forward<<std::endl;

	rVel = -rVel * maxV * forward * (i/10);
	lVel = lVel * maxV * forward * (i/10);


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
	ros::Subscriber joySub = nh.subscribe("/joy",10,&joyCallBack);

	
	//ros::init(argc, argv, "talker");
	//ros::NodeHandle n;
	ros::Publisher chatter_pub = nh.advertise<epos::wheel_drive>("/velocities", 1000);
	ros::Rate rate(20);
	epos::wheel_drive msg;
	msg.motorIDs = {1,2,3,4};
	while(ros::ok())
	{
	//msg.driveMode = rVel;
	//std::cout << msg.velocities << std::endl;
	msg.velocities = {lVel, rVel, lVel, rVel};
	chatter_pub.publish(msg);
	
	rate.sleep();
	ros::spinOnce();
	}
}



