/*
	Maxon Motor Controller epos_cmd
	epos_cmd.h
	Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

	@author Jared Beard
	@version 1.0 11/13/18
*/
#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

  #ifndef MMC_SUCCESS
	 #define MMC_SUCCESS 0
  #endif

  #ifndef MMC_FAILED
	 #define MMC_FAILED 1
  #endif

  #ifndef MMC_MAX_LOG_MSG_SIZE
	 #define MMC_MAX_LOG_MSG_SIZE 512
  #endif

#include <ros/ros.h>
#include <ros/console.h>
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

typedef void* HANDLE;
typedef int BOOL;

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

class epos_cmd {
  void* pKeyHandle = 0;
  //unsigned short maxStrSize = 512;

  std::vector<unsigned short> usNodeId;

  std::string deviceName;
  std::string protocolStackName;
  std::string interfaceName;
  std::string portName;
  int baudrate;
  int NumDevices;
  int result;
  unsigned int *errorCode;
  char* errorCodeChar;


public:
    /***********************ENUMS*********************************/
  enum OpMode {
    OMD_PROFILE_POSITION_MODE = 1,
    OMD_PROFILE_VELOCITY_MODE = 3,
    OMD_HOMING_MODE = 6,
    OMD_CURRENT_MODE = -3
  };
  enum DevState {
    DISABLED = 0x0000,
    ENABLED = 0x0001,
    QUICKSTOP = 0x0002,
    FAULT = 0x0003
  };

  /***********************VARIABLES*****************************/
  DevState current_state;

  //Setup error codes to print intead of being accepted as input. Makes the output simpler...
  /***********************INTIALIZATION***************************/
  int OpenDevices   ();
  int CloseDevices  ();


  /***********************CONFIGURATION***************************/
  int setMode(unsigned short nodeID, OpMode mode);
  //void setModeCallback(const ; //NEED TO MAKE MESSAGE FOR THIS, need to make way to display/handle specific error
  int resetDevice(unsigned short nodeID);
  int setState(unsigned short nodeID, DevState state);
  int getState(unsigned short nodeID, DevState state);

  /***********************OPERATION*******************************/


  /***********************PRINT/DEBUGGING*************************/
  int getError(unsigned short errorCodeValue); //NEED to convert error code to text
  void LogError(std::string functionName, int result);



  /***********************CONSTRUCTORS****************************/
  epos_cmd(); // Set motor type, sensor types, max following error, max velocity, max acc,
                    // velocity units, default operation mode
  epos_cmd(std::vector<int> ids, int br);
  //motor_cmd(); <- read input from launch
  ~epos_cmd(){}





/**  int   handleFault(); // get
  int   getCondition();
  int   setPosProfile();
  int   goToPos();
  int   stopPos();
  int   setVelProfile();
  int   gotoVel();
  int   stopVel();
  int   startHomingMode();
  int   setHome();
  int   goToHome();
  int   stopHome();
  int   waitForHome();
  int   setCurrentMust();
  int   getCurrentMust();*/



private:
  /***********************VARIABLES*****************************/

  /***********************FUNCTIONS*****************************/
  short unsigned int getDevStateValue(DevState state);




//  int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
//  int   Demo(unsigned int* pErrorCode);
//  int   PrepareDemo(unsigned int* pErrorCode);


//  void cmdReceived(const geometry_msgs::Twist& msg);
//  void clearFaultCallback(const sensor_msgs::Joy& msg);

  /***************Print and Commands*****************/

//  int   PrepareMotor(unsigned int* pErrorCode, unsigned short int nodeId);

};
#endif
