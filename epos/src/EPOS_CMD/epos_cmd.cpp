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
int epos_cmd::openDevices()
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

		//std::cout << "dev " << pDeviceName << "__Prot " << pProtocolStackName << "__Int " << pInterfaceName << "__Por" << pPortName << "__Code" << errorCode << std::endl;

		ROS_INFO("Open device...");
		//Opens device
		keyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &errorCode);
		//checking device opened
		if(keyHandle!=0 && errorCode == 0)
		{
				unsigned int lBaudrate = 0;
				unsigned int lTimeout = 0;

				if(VCS_GetProtocolStackSettings(keyHandle, &lBaudrate, &lTimeout, &errorCode))
				{
						if(VCS_SetProtocolStackSettings(keyHandle, baudrate, lTimeout, &errorCode))
						{
								if(VCS_GetProtocolStackSettings(keyHandle, &lBaudrate, &lTimeout, &errorCode))
								{
										if(baudrate==(int)lBaudrate)
										{
												result = MMC_SUCCESS;
												ROS_INFO("Device opened");
										}
								}
						}
				}
		}
		else
		{
				keyHandle = 0;
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
int epos_cmd::closeDevices()
{
		int result = MMC_FAILED;
		errorCode = 0;
		ROS_INFO("Close device");

		if(VCS_CloseAllDevices(&errorCode) && errorCode == 0)
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
int epos_cmd::setMode(std::vector<int> IDs, OpMode mode)
{
		current_mode = mode;
		for (int i = 0; i < IDs.size(); ++i)
		{
				//std::cout << IDs.size() << std::endl;
				if(VCS_SetOperationMode(keyHandle, IDs[i], mode, &errorCode))
				{
						ROS_INFO("Operation mode set");
				} else
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

		if (VCS_ResetDevice(keyHandle, nodeID, &errorCode))
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
		if( current_state == state || VCS_SetState(keyHandle,nodeID,state,&errorCode))
		{
				return MMC_SUCCESS;
		} else
		{
				return MMC_FAILED;
		}
}

/**
    Gets Device State in state machine

    @param nodeID node to have operation mode modified
    @param state desried state of state machine
    @return Success(0)/Failure(1) of command
 */
int epos_cmd::getState(unsigned short nodeID, DevState &state)
{
		short unsigned int stateValue; // = getDevStateValue(state);
		ROS_DEBUG("Retrieving State");
		if( VCS_GetState(keyHandle,nodeID,&stateValue,&errorCode))
		{
				ROS_DEBUG("State Retrieved");
				state = getDevState(stateValue);
				//std::cout << "Motor "<< nodeID << " is in state " << state << std::endl;
				return MMC_SUCCESS;

		} else {
				ROS_WARN("State Failed");
				return MMC_FAILED;
		}
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

int epos_cmd::getModeValue(OpMode mode){
		int position = 1;
		int velocity = 3;
		int homing = 6;
		int current = -3;

		if (mode == position) {
				return position;
		} else if (mode == velocity) {
				return velocity;
		} else if (mode == homing) {
				return homing;
		} else if (mode == current) {
				return current;
		} else {
				std::cout << "Invalid OpMode" << std::endl;
				return 8;
		}
}

enum epos_cmd::DevState epos_cmd::getDevState(short unsigned int state)
{
		short unsigned int disabled = 0x0000;
		short unsigned int enabled = 0x0001;
		short unsigned int quickstop = 0x0002;
		short unsigned int fault = 0x0003;

		if (state == disabled) {
				return DISABLED;
		} else if (state == enabled) {
				return ENABLED;
		} else if (state == quickstop) {
				return QUICKSTOP;
		} else if (state == fault) {
				return FAULT;
		} else {
				ROS_WARN("Invalid DevStateValues");
				return FAULT;
		}
}

int epos_cmd::handleFault(int ID)
{
		BOOL isFault = 0;
		std::cout << "HF start" << std::endl;
		if(VCS_GetFaultState(keyHandle, ID, &isFault, &errorCode ))
		{
				if(isFault)
				{
						logError("VCS_GetFaultState");
						if(VCS_ClearFault(keyHandle, ID, &errorCode) )
						{
								ROS_INFO("Fault Cleared");
								return MMC_SUCCESS;
						} else
						{
								logError("VCS_ClearFault");
								ROS_INFO("Failed Fault Clear");
								return MMC_FAILED;
						}
				} else
				{
						ROS_DEBUG("No fault motor");
				}
		} else
		{
				ROS_WARN("Get Fault state failed");
		}
}

int epos_cmd::prepareMotors(std::vector<int> IDs)
{
		DevState state;
		for (int i = 0; i < IDs.size(); ++i)
		{
				//std::cout << "Prepare Motors "<< std::endl;
				if (getState(IDs[i], state))
				{
						//std::cout << "State " << state << ";P" << std::endl;
						if (state == FAULT)
						{
								std::cout << "FAULT" << std::endl;
								handleFault(IDs[i]);
								std::cout << "Clear Fault " << IDs[i] << std::endl;
						}
						if (state != ENABLED)
						{
								setState(IDs[i], ENABLED);
						}
				} else
				{
						ROS_WARN("Get state failed: motor %d", IDs[i]);
				}
		}
		return MMC_SUCCESS;
}

int epos_cmd::goToVel(std::vector<int> IDs, std::vector<long> velocities)
{
		if (current_mode != OMD_PROFILE_VELOCITY_MODE) setMode(IDs, OMD_PROFILE_VELOCITY_MODE);

		for (int i = 0; i < IDs.size(); ++i)
		{
				if (abs(velocities[i]) > 0)
				{
						if (!VCS_MoveWithVelocity(keyHandle, IDs[i], velocities[i],&errorCode))
						{
								logError("VCS_MoveWithVelocity");
								return MMC_FAILED;
						} else {
								ROS_INFO("Running");
						}
				} else
				{
						if (!VCS_HaltVelocityMovement(keyHandle, IDs[i],&errorCode))
						{
								return MMC_FAILED;
						}
				}
		}
		return MMC_SUCCESS;
}

int epos_cmd::getPosition(std::vector<int> IDs, std::vector<int> &positions) //128 cts/turn
{
		int pos = 0;
		//ROS_WARN("---------------------------------------------------%d", pos);
		//*pos = 1;
		//ROS_WARN("---------------------------------------------------%d", *pos);
		for (int i = 0; i < IDs.size(); ++i)
		{
				ROS_WARN("pos %d",  IDs[i]);
				if (VCS_GetPositionIs(keyHandle, IDs[i], &pos, &errorCode))
				{
						ROS_WARN(" is %d", pos);
						positions.push_back(pos);
						std::cout << " is " << pos << std::endl;
				}
				else
				{
						std::cout << " FAILED POSITION. " << std::endl;
						return MMC_FAILED;
				}
		}

		return MMC_SUCCESS;
}

int epos_cmd::goToTorque(std::vector<int> IDs, std::vector<long> torques, double gr)
{
		if (current_mode != OMD_CURRENT_MODE) setMode(IDs, OMD_CURRENT_MODE);

		for (int i = 0; i < IDs.size(); ++i)
		{
				short currentA = floor(torques[i]/(kT*gr));
				if (VCS_SetCurrentMust(keyHandle, IDs[i], currentA,&errorCode))
				{
						ROS_INFO("Running Current");
				} else {
						logError("VCS_SetCurrentMust");
						return MMC_FAILED;
				}

		}
		return MMC_SUCCESS;
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


void epos_cmd::logError(std::string functionName)
{
		std::cerr << "EPOS COMMAND: " << functionName << " failed (errorCode=0x" << std::hex << errorCode << ")"<< std::endl;
}


int epos_cmd::checkNodeID(int ID)
{
		int result = MMC_FAILED;

		for (int i = 0; i < nodeIDList.size(); ++i)
		{
				if (ID == nodeIDList[i]) {
						result == MMC_SUCCESS;
				}
		}
		return result;
}
/////////////////////////////////////////////////////////////////////
/***************************CONSTRUCTORS****************************/
/////////////////////////////////////////////////////////////////////
/**
    Default Constructor
 */
epos_cmd::epos_cmd(){
		nodeIDList.push_back(2);
		deviceName = "EPOS4";
		protocolStackName = "MAXON SERIAL V2";
		interfaceName = "USB";
		portName = "USB0";
		baudrate = 1000000;
		NumDevices = 1;

		ROS_INFO("EPOS COMMAND START");
}

/**
    Constructor for specific motor ids and baudrate

    @param ids id number of motors to be used
    @param br baudrate for communications
 */
epos_cmd::epos_cmd(std::vector<int> ids, int br){
		for (int i = 0; i < ids.size(); ++i) {
				nodeIDList.push_back( (unsigned short) ids[i]);

		}
		deviceName = "EPOS4";
		protocolStackName = "MAXON SERIAL V2";
		interfaceName = "USB";
		portName = "USB0";
		baudrate = br;
		NumDevices = ids.size();

		ROS_INFO("EPOS COMMAND START");
}

/**
    Destructor closes all devices
 */
epos_cmd::~epos_cmd()
{
		int i = 0;
		while(!closeDevices() && i < 5)
		{
				std::cerr << "Failed to close devices, try " << i << "."<< std::endl;
				++i;
		};
		if (i == 5) {
				std::cerr << "Failed: Aborting device closure" << std::endl;
		}
}
