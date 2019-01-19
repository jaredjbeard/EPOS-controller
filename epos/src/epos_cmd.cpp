/*
	Maxon Motor Controller epos_cmd
	epos_cmd.cpp
	Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

	@author Jared Beard
	@version 1.0 11/13/18
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <epos/epos_cmd.h>
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


/////////////////////////////////////////////////////////////////////
/***************************INITIALIZATION**************************/
/////////////////////////////////////////////////////////////////////
/**
		Opens device and subdevices

		@param pErrorCode Pointer to error code as output by Maxon codes
		@return Success(0)/Failure(1) of commands
*/
int epos_cmd::OpenDevices(unsigned int* pErrorCode)
{
	//Success of code
	int result = MMC_FAILED;
	// Internal use of motor parameters
	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, deviceName.c_str());
	strcpy(pProtocolStackName, protocolStackName.c_str());
	strcpy(pInterfaceName, interfaceName.c_str());
	strcpy(pPortName, portName.c_str());

	ROS_INFO("Open device...");
	//Opens device
	pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, pErrorCode);
	//checking device opened
	if(pKeyHandle!=0 && *pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(pKeyHandle, &lBaudrate, &lTimeout, pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(pKeyHandle, baudrate, lTimeout, pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(pKeyHandle, &lBaudrate, &lTimeout, pErrorCode)!=0)
				{
					if(baudrate==(int)lBaudrate)
					{
						result = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		pKeyHandle = 0;
	}
	// remove temporary device std::strings
	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	//Get initial device state here

	return result;
}

/**
		Closes device and subdevices

		@param pErrorCode Pointer to error coud output by Maxon commands
  	@return Success(0)/Failure(1) of command
*/
int epos_cmd::CloseDevices(unsigned int* pErrorCode)
{
	int result = MMC_FAILED;
	*pErrorCode = 0;

	ROS_INFO("Close device");

	if(VCS_CloseAllDevices(pErrorCode)!=0 && *pErrorCode == 0)
	{
		result = MMC_SUCCESS;
	}
	return result;
}

/////////////////////////////////////////////////////////////////////
/***************************CONFIGURATION***************************/
/////////////////////////////////////////////////////////////////////
/**
		Closes device and subdevices

		@param nodeID node to have operation mode modified
		@param mode operation mode to be set
		@param pErrorCode Pointer to error coud output by Maxon commands
  	@return Success(0)/Failure(1) of command
*/
int   epos_cmd::setMode(unsigned short nodeID, OpMode mode, unsigned int* pErrorCode)
{
	result = MMC_FAILED;

	if(VCS_SetOperationMode(pKeyHandle, nodeID, mode, pErrorCode))
	{
		result = MMC_SUCCESS;
	}
	return result;
}

/**
		ROS callback for setting operational mode

		@param msg ROS msg of custom type:
*/
void   epos_cmd::setModeCallback(const geometry_msgs::Twist& msg)
{
	unsigned int* pErrorCode = 0;
	for(int i = 0; i < msg->nodeId.size(); ++i)
	{
  	if (!setMode(msg->nodeID[i],msg->mode, pErrorCode))
		{
			ROS_ERROR('FAILED TO SET MODE OF NODE %i', nodeID[i])
			break;
		}
	}
}

/**
		Resets device state machine

    @param nodeID node to have operation mode modified
		@param pErrorCode Pointer to error coud output by Maxon commands
    @return Success(0)/Failure(1) of command
*/
int   epos_cmd::resetDevice(unsigned short nodeID, unsigned int* pErrorCode)
{
	int result = MMC_FAILED;

	if (VCS_ResetDevice(pKeyHandle, nodeID, pErrorCode))
	{
		result = MMC_SUCCESS;
	}

	return result;
}

/**
		Sets Device State in state machine

    @param nodeID node to have operation mode modified
		@param state desried state of state machine
		@param pErrorCode Pointer to error coud output by Maxon commands
    @return Success(0)/Failure(1) of command
*/
int epos_cmd::setState(unsigned short nodeID, DevState state, unsigned int* pErrorCode)
{
	int result = MMC_FAILED;

	if( current_state == state || VCS_SetState(pKeyHandle,nodeID,state,pErrorCode))
	{
		result = MMC_SUCCESS;
	}

	return result;
}

/**
		Gets Device State in state machine

    @param nodeID node to have operation mode modified
		@param state desried state of state machine
		@param pErrorCode Pointer to error coud output by Maxon commands
    @return Success(0)/Failure(1) of command
*/
int epos_cmd::setState(unsigned short nodeID, DevState* state, unsigned int* pErrorCode)
{
	int result = MMC_FAILED;

	if( VCS_GetState(pKeyHandle,nodeID,state,pErrorCode))
	{
		result = MMC_SUCCESS;
	}

	return result;
}



/////////////////////////////////////////////////////////////////////
/***************************OPERATION*******************************/
/////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////
/***************************PRINT/DEBUGGING*************************/
/////////////////////////////////////////////////////////////////////
/**
		Displays Error info for an executed function

    @param ErrorCodeValue Error code number
		@param pErrorCode Pointer to error coud output by Maxon commands
    @return Success(0)/Failure(1) of command
*/
int getError(unsigned short ErrorCodeValue, char* pErrorCode)
{
  int result = MMC_FAILED;
  if(VCS_GetErrorInfo(ErrorCodeValue, pErrorCode, maxStrSize))
  {
    ROS_ERROR("ERROR %u: %s\n", ErrorCodeValue, *pErrorCode);
		reult = MMC_SUCCESS
  }

	return result;
}


void LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

/////////////////////////////////////////////////////////////////////
/***************************CONSTRUCTORS****************************/
/////////////////////////////////////////////////////////////////////
/**
    Default Constructor
*/
epos_cmd::epos_cmd(){
  usNodeId.push_back(2);
  pKeyHandle = 0;
  deviceName = "EPOS4";
  protocolStackName = "MAXON SERIAL V2";
  interfaceName = "USB";
  portName = "USB0";
  baudrate = 1000000;
  NumDevices = 1;
  maxStrSize = 512;

  PrintHeader();
}

/**
		Constructor for specific motor ids and baudrate

		@param ids id number of motors to be used
		@param br baudrate for communications
    @param maxstd::stringSize Maximum length of std::string for error codes
*/
epos_cmd(std::vector<int> ids, int br, int maxstd::stringSize){
  for (int i = 1: i < ids.size(): ++i){
    usNodeId.push_back( (unsigned short) ids[i]);

  }
  pKeyHandle = 0;
  deviceName = "EPOS4";
  protocolStackName = "MAXON SERIAL V2";
  interfaceName = "USB";
  portName = "USB0";
  baudrate = br;
  NumDevices = ids.size();
  maxStrSize = maxstd::stringSize;

  PrintHeader();
}
