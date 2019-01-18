/*
	Maxon Motor Controller motor_cmd
	motor_cmd.h
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
#include <epos/Definitions.h>
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


class motor_cmd {
  typedef void* HANDLE;
  typedef int BOOL;

  enum EAppMode {
  	AM_UNKNOWN,
  	AM_DEMO,
  	AM_INTERFACE_LIST,
  	AM_PROTOCOL_LIST,
  	AM_VERSION_INFO
  };

  const string g_programName = "HelloEposCmd";
  //void* g_pKeyHandle = 0;
  EAppMode g_eAppMode = AM_DEMO;

public:
  int   OpenDevice(unsigned int* p_pErrorCode);
  int   CloseDevices(unsigned int* p_pErrorCode);
  int   ParseArguments(int argc, char** argv);



  /*****************CONSTRUCTORS*********************/
  void motor_cmd();
  void motor_cmd(std::vector<int> ids, int br);
  //motor_cmd();
  void ~motor_cmd();

private:
  std::vector<unsigned short> g_usNodeId;
  HANDLE g_pKeyHandle;
  std::vector<HANDLE> g_pSubKeyHandles;
  string g_deviceName;
  string g_protocolStackName;
  string g_interfaceName;
  string g_portName;
  int g_baudrate;
  int NumDevices;



  int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
  int   Demo(unsigned int* p_pErrorCode);
  int   PrepareDemo(unsigned int* p_pErrorCode);
  int   PrintAvailableInterfaces();
  int	  PrintAvailablePorts(char* p_pInterfaceNameSel);
  int	  PrintAvailableProtocols();
  int   PrintDeviceVersion();

  void cmdReceived(const geometry_msgs::Twist& msg);
  void clearFaultCallback(const sensor_msgs::Joy& msg);

  /***************Print and Commands*****************/
  void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
  void  PrintSettings();
  void  PrintUsage();
  void  SeparatorLine();
  void  PrintHeader();
  void  PrintSettings();

};
#endif
