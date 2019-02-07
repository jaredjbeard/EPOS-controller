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
int epos_cmd::setMode(std::vector<unsigned short> nodeIDs, OpMode mode)
{
		for (int i = 0; i < nodeIDs.size(); ++i)
		{
				if(!VCS_SetOperationMode(pKeyHandle, nodeIDs[i], mode, errorCode))
				{
						logError("SetOperationMode");
						return MMC_FAILED;
				}
		}
		return MMC_SUCCESS;
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

		if (state == DISABLED) {
				return disabled;
		} else if (state == ENABLED) {
				return enabled;
		} else if (state == QUICKSTOP) {
				return quickstop;
		} else if (state == FAULT) {
				return fault;
		} else {
				std::cout << "Invalid DevState" << std::endl;
				return 8;
		}
}

int epos_cmd::handleFault(std::vector<int> nodeIDs)
{
		int result = MMC_SUCCESS;
		for (int i = 0; i < nodeIDs.size(); ++i)
		{
				BOOL isFault = 0;
				if(VCS_GetFaultState(pKeyHandle, nodeIDs[i], &isFault, errorCode ) == 0)
				{
						logError("VCS_GetFaultState");
						if(isFault == true && VCS_ClearFault(pKeyHandle, nodeIDs[i], errorCode) == 0)
						{
								logError("VCS_ClearFault");
								result = MMC_FAILED;
								return result;
						}
				}
		}
		return result;
}
/**
   int PrepareMotor(unsigned int* p_pErrorCode, unsigned short int nodeId)
   {
    int lResult = MMC_SUCCESS;

    if(lResult==0)
    {
      BOOL oIsEnabled = 0;

      if(VCS_GetEnableState(g_pKeyHandle, nodeId, &oIsEnabled, p_pErrorCode) == 0)
      {
        logError("VCS_GetEnableState", lResult, *p_pErrorCode);
        lResult = MMC_FAILED;
      }

      if(lResult==0)
      {
        if(!oIsEnabled)
        {
          if(VCS_SetEnableState(g_pKeyHandle, nodeId, p_pErrorCode) == 0)
          {
            logError("VCS_SetEnableState", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
          }
        }
      }
    }
   }
   return lResult;
   }

   /**bool ProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long Velocity)
   {
   int lResult = MMC_SUCCESS;
   std::stringstream msg;
    long targetvelocity = Velocity;
    if (p_usNodeId > 3)
        targetvelocity = -targetvelocity;
   msg << "set profile velocity mode, node = " << p_usNodeId;

   logInfo(msg.str());

   if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
   {
    logError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
    lResult = MMC_FAILED;
   }
   else
   {
    stringstream msg;
    msg << "move with target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
    logInfo(msg.str());

    if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
    {
      lResult = MMC_FAILED;
      logError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
    }
   }

   return lResult;
   }


   int PrepareDemo(unsigned int* p_pErrorCode)
   {
   int lResult = MMC_SUCCESS;
   for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
   {
      lResult = PrepareMotor(p_pErrorCode, deviceNum);
      if (lResult != MMC_SUCCESS)
      {
          return lResult;
      }
   }
   return lResult;
   }

   int Demo(unsigned int* p_pErrorCode)
   {
   int lResult = MMC_SUCCESS;
   unsigned int lErrorCode = 0;
    for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
    {
        long Velocity = 2000;
      lResult = ProfileVelocityMode(g_pKeyHandle, deviceNum, lErrorCode, Velocity);
      if(lResult != MMC_SUCCESS)
      {
        logError("DemoProfileVelocityMode", lResult, lErrorCode);
      }
   }
   sleep(5);
      for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
    {
      lResult = HaltVelocity(g_pKeyHandle, deviceNum, lErrorCode);
      if(lResult != MMC_SUCCESS)
      {
        logError("DemoProfileVelocityMode", lResult, lErrorCode);
      }
   }
   for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
   {
      if(VCS_SetDisableState(g_pKeyHandle, deviceNum, &lErrorCode) == 0)
      {
        logError("VCS_SetDisableState", lResult, lErrorCode);
        lResult = MMC_FAILED;
      }
   }

   return lResult;
   }*/

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


void epos_cmd::logError(std::string functionName)
{
		std::cerr << "EPOS COMMAND: " << functionName << " failed (errorCode=0x" << std::hex << errorCode << ")"<< std::endl;
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
		for (int i = 0; i < ids.size(); ++i) {
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

/**
    Destructor closes all devices
 */
epos_cmd::~epos_cmd()
{
	int i = 0;
		while(!CloseDevices() && i < 5)
		{
			std::cerr << "Failed to close devices, try " << i << "."<< std::endl;
			++i;
		};
		if (i == 5){
			std::cerr << "Failed: Aborting device closure" << std::endl;
		}
}
