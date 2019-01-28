/*
	Maxon Motor Controller epos_cmd
	epos_cmd.cpp
	Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

	@author Jared Beard
	@version 1.0 11/13/18
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <Definitions.h>
#include <epos/epos_cmd.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
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

		@return Success(0)/Failure(1) of commands
*/
int epos_cmd::OpenDevices()
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
	pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, errorCode);
	//checking device opened
	if(pKeyHandle!=0 && *errorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(pKeyHandle, &lBaudrate, &lTimeout, errorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(pKeyHandle, baudrate, lTimeout, errorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(pKeyHandle, &lBaudrate, &lTimeout, errorCode)!=0)
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

  	@return Success(0)/Failure(1) of command
*/
int epos_cmd::CloseDevices()
{
	int result = MMC_FAILED;
	*errorCode = 0;

	ROS_INFO("Close device");

	if(VCS_CloseAllDevices(errorCode)!=0 && *errorCode == 0)
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
  	@return Success(0)/Failure(1) of command
*/
int epos_cmd::setMode(unsigned short nodeID, OpMode mode)
{
	result = MMC_FAILED;

	if(VCS_SetOperationMode(pKeyHandle, nodeID, mode, errorCode))
	{
		result = MMC_SUCCESS;
	}
	return result;
}

/**
		ROS callback for setting operational mode

		@param msg ROS msg of custom type:
*/
/**void epos_cmd::setModeCallback(const )
{
	for(int i = 0; i < msg.nodeID.size(); ++i)
	{
  	if (!setMode(msg.nodeID[i],msg.mode, errorCode))
		{
			ROS_ERROR("FAILED TO SET MODE OF NODE %d", msg.nodeID[i]);
			break;
		}
	}
}

/**
		Resets device state machine

    @param nodeID node to have operation mode modified

    @return Success(0)/Failure(1) of command
*/
int epos_cmd::resetDevice(unsigned short nodeID)
{
	int result = MMC_FAILED;

	if (VCS_ResetDevice(pKeyHandle, nodeID, errorCode))
	{
		result = MMC_SUCCESS;
	}

	return result;
}

/**
		Sets Device State in state machine

    @param nodeID node to have operation mode modified
		@param state desried state of state machine
    @return Success(0)/Failure(1) of command
*/
int epos_cmd::setState(unsigned short nodeID, DevState state)
{
	int result = MMC_FAILED;

	if( current_state == state || VCS_SetState(pKeyHandle,nodeID,state,errorCode))
	{
		result = MMC_SUCCESS;
	}

	return result;
}

/**
		Gets Device State in state machine

    @param nodeID node to have operation mode modified
		@param state desried state of state machine
    @return Success(0)/Failure(1) of command
*/
int epos_cmd::getState(unsigned short nodeID, DevState state)
{
	int result = MMC_FAILED;
	short unsigned int stateValue = getDevStateValue(state);

	if( VCS_GetState(pKeyHandle,nodeID,&stateValue,errorCode))
	{
		result = MMC_SUCCESS;
	}

	return result;
}



/////////////////////////////////////////////////////////////////////
/***************************OPERATION*******************************/
/////////////////////////////////////////////////////////////////////


short unsigned int epos_cmd::getDevStateValue(DevState state){
		short unsigned int disabled = 0x0000;
		short unsigned int enabled = 0x0001;
		short unsigned int quickstop = 0x0002;
		short unsigned int fault = 0x0003;

		if (state == DISABLED){
			return disabled;
		} else if (state == ENABLED){
			return enabled;
		} else if (state == QUICKSTOP){
			return quickstop;
		} else if (state == FAULT){
			return fault;
		} else {
			std::cout << "Invalid DevState" << std::endl;
			return 8;
		}
}

/////////////////////////////////////////////////////////////////////
/***************************PRINT/DEBUGGING*************************/
/////////////////////////////////////////////////////////////////////
/**
		Displays Error info for an executed function

    @param ErrorCodeValue Error code number
    @return Success(0)/Failure(1) of command
*/
int epos_cmd::getError(unsigned short errorCodeValue)
{
  int result = MMC_FAILED;
  if(VCS_GetErrorInfo(errorCodeValue, errorCodeChar, MMC_MAX_LOG_MSG_SIZE))
  {
    ROS_ERROR("ERROR %u: %u\n", errorCodeValue, *errorCodeChar);
		result = MMC_SUCCESS;
  }

	return result;
}


void epos_cmd::LogError(std::string functionName, int result)
{
	std::cerr << "EPOS COMMAND: " << functionName << " failed (result=" << result << ", errorCode=0x" << std::hex << errorCode << ")"<< std::endl;
}

/////////////////////////////////////////////////////////////////////
/***************************CONSTRUCTORS****************************/
/////////////////////////////////////////////////////////////////////
/**
    Default Constructor
*/
epos_cmd::epos_cmd(){
  usNodeId.push_back(2);
  deviceName = "EPOS4";
  protocolStackName = "MAXON SERIAL V2";
  interfaceName = "USB";
  portName = "USB0";
  baudrate = 1000000;
  NumDevices = 1;

	std::cout << "EPOS COMMAND START" << std::endl;
}

/**
		Constructor for specific motor ids and baudrate

		@param ids id number of motors to be used
		@param br baudrate for communications
*/
epos_cmd::epos_cmd(std::vector<int> ids, int br){
  for (int i = 0; i < ids.size(); ++i){
    usNodeId.push_back( (unsigned short) ids[i]);

  }
  deviceName = "EPOS4";
  protocolStackName = "MAXON SERIAL V2";
  interfaceName = "USB";
  portName = "USB0";
  baudrate = br;
  NumDevices = ids.size();

	std::cout << "EPOS COMMAND START" << std::endl;
}
