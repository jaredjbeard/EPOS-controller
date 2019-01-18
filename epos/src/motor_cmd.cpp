/*
	Maxon Motor Controller motor_cmd
	motor_cmd.cpp
	Purpose: Wrap EPOS commands and integrate with ROS for general purpose motor control

	@author Jared Beard
	@version 1.0 11/13/18
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <epos/motor_cmd.h>
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

/**
		Opens device and subdevices

		@param p_pErrorCode Pointer to error code as output by Maxon codes
		@return Success(0)/Failure(1) of commands
*/
int motor_cmd::OpenDevice(unsigned int* p_pErrorCode)
{
	//Success of code
	int lResult = MMC_FAILED;
	// Internal use of motor parameters
	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	ROS_INFO("Open device...");
	//Opens device
	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);
	//checking device opened
	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}
	// remove temporary device strings
	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

/**
		Closes device and subdevices

		@param p_pErrorCode Pointer to error coud output by Maxon commands
  	@return Success(0)/Failure(1) of command
*/
int motor_cmd::CloseDevices(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;
	*p_pErrorCode = 0;

	ROS_INFO("Close device");

	if(VCS_CloseAllDevices(p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}
	return lResult;
}


/**
		Set device to run on profile position mode (See Maxon Documentation)

		@param p_DeviceHandle handle of device to operate on
		@param p_usNodeId id of motor controllers (nodes)
		@param p_rlErrorCode Error code from Maxon commands
    @return Success(0)/Failure(1)
*/
int motor_cmd::startPPositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	ROS_INFO(msg.str());
//put in own function
	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

/**
		Set device to run on profile position mode (See Maxon Documentation)

		@param p_DeviceHandle handle of device to operate on
		@param p_usNodeId id of motor controllers (nodes)
		@param p_rlErrorCode Error code from Maxon commands
    @return Success(0)/Failure(1)
*/
int motor_cmd::runPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long targetPosition)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
	msg << "move to position = " << targetPosition << ", node = " << p_usNodeId;
	ROS_INFO(msg.str());

	if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 0, 1, &p_rlErrorCode) == 0)
	{
		LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
		break;
	}
}

/**
		Set device to run on profile position mode (See Maxon Documentation)

		@param p_DeviceHandle handle of device to operate on
		@param p_usNodeId id of motor controllers (nodes)
		@param p_rlErrorCode Error code from Maxon commands
    @return Success(0)/Failure(1)
*/
int motor_cmd::haltPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;

	ROS_INFO("halt position movement");

	if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
}

/**
		Set device to run on profile position mode (See Maxon Documentation)

		@param p_DeviceHandle handle of device to operate on
		@param p_usNodeId id of motor controllers (nodes)
		@param p_rlErrorCode Error code from Maxon commands
    @return Success(0)/Failure(1)
*/
int motor_cmd::DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	ROS_INFO(msg.str());
//put in own function
	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
		list<long> positionList;

		positionList.push_back(5000);
		positionList.push_back(-10000);
		positionList.push_back(5000);

		for(list<long>::iterator it = positionList.begin(); it !=positionList.end(); it++)
		{
			long targetPosition = (*it);
			stringstream msg;
			msg << "move to position = " << targetPosition << ", node = " << p_usNodeId;
			ROS_INFO(msg.str());
			//put in own function
			if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 0, 1, &p_rlErrorCode) == 0)
			{
				LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
				break;
			}

			sleep(1);
		}

		if(lResult == MMC_SUCCESS)
		{
			ROS_INFO("halt position movement");

			if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}
		}
	}

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
bool motor_cmd::ProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long Velocity)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
    long targetvelocity = Velocity;
    if (p_usNodeId > 3)
        targetvelocity = -targetvelocity;
	msg << "set profile velocity mode, node = " << p_usNodeId;

	ROS_INFO(msg.str());

	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
		stringstream msg;
		msg << "move with target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
		ROS_INFO(msg.str());

		if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
		{
			lResult = MMC_FAILED;
			LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
		}
	}

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
bool motor_cmd::SetVelocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long Velocity)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;
    long targetvelocity = Velocity;
    if (p_usNodeId > 3)
        targetvelocity = -targetvelocity;
	msg << "set profile velocity mode, node = " << p_usNodeId;
	ROS_INFO(msg.str());

		msg << "move with target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
		ROS_INFO(msg.str());

		if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
		{
			lResult = MMC_FAILED;
			LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
		}

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
bool motor_cmd::ActivateProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile velocity mode, node = " << p_usNodeId;

	ROS_INFO(msg.str());

	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
bool motor_cmd::HaltVelocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
			ROS_INFO("halt velocity movement");
			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
			}
	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::PrepareMotor(unsigned int* p_pErrorCode, unsigned short int nodeId)
{
    int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, nodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << nodeId << "'";
			ROS_INFO(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, nodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, nodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, nodeId, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::PrepareDemo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;
	for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
	{
	    lResult = PrepareMotor(p_pErrorCode, deviceNum);
	    if (lResult != MMC_SUCCESS)
	    {
	        return lResult;
	    }
	    lResult = ActivateProfileVelocityMode(g_pKeyHandle, deviceNum, lErrorCode);
	    if (lResult != MMC_SUCCESS)
	    {
	    	LogError("ActivateProfileVelocityMode", lResult, lErrorCode);
	        return lResult;
	    }
	}

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::DisableMotor(unsigned int* p_pErrorCode, unsigned short int nodeId)
{
    int lResult = MMC_SUCCESS;
    unsigned int lErrorCode = 0;
	for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
	{
			if(VCS_SetDisableState(g_pKeyHandle, deviceNum, &lErrorCode) == 0)
			{
				LogError("VCS_SetDisableState", lResult, lErrorCode);
				lResult = MMC_FAILED;
			}
	}
	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::DisableDemo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
	{
	    lResult = DisableMotor(p_pErrorCode, deviceNum);
	    if (lResult != MMC_SUCCESS)
	    {
	        return lResult;
	    }
	}
	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::Demo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;
    for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
    {
        long Velocity = 2000;
	    lResult = ProfileVelocityMode(g_pKeyHandle, deviceNum, lErrorCode, Velocity);
	    if(lResult != MMC_SUCCESS)
	    {
	    	LogError("DemoProfileVelocityMode", lResult, lErrorCode);
	    }
	}
	sleep(5);
	    for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
    {
	    lResult = HaltVelocity(g_pKeyHandle, deviceNum, lErrorCode);
	    if(lResult != MMC_SUCCESS)
	    {
	    	LogError("DemoProfileVelocityMode", lResult, lErrorCode);
	    }
	}
	for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
	{
			if(VCS_SetDisableState(g_pKeyHandle, deviceNum, &lErrorCode) == 0)
			{
				LogError("VCS_SetDisableState", lResult, lErrorCode);
				lResult = MMC_FAILED;
			}
	}

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::PrintAvailablePorts(char* p_pInterfaceNameSel)
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), p_pInterfaceNameSel, lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetPortNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;
			printf("            port = %s\n", pPortNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::PrintAvailableInterfaces()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pInterfaceNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetInterfaceNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), lStartOfSelection, pInterfaceNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetInterfaceNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("interface = %s\n", pInterfaceNameSel);

			PrintAvailablePorts(pInterfaceNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pInterfaceNameSel;

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::PrintDeviceVersion()
{
	int lResult = MMC_FAILED;
	unsigned short usHardwareVersion = 0;
	unsigned short usSoftwareVersion = 0;
	unsigned short usApplicationNumber = 0;
	unsigned short usApplicationVersion = 0;
	unsigned int ulErrorCode = 0;

	if(VCS_GetVersion(g_pKeyHandle, g_usNodeId, &usHardwareVersion, &usSoftwareVersion, &usApplicationNumber, &usApplicationVersion, &ulErrorCode))
	{
		printf("%s Hardware Version    = 0x%04x\n      Software Version    = 0x%04x\n      Application Number  = 0x%04x\n      Application Version = 0x%04x\n",
				g_deviceName.c_str(), usHardwareVersion, usSoftwareVersion, usApplicationNumber, usApplicationVersion);
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
int motor_cmd::PrintAvailableProtocols()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pProtocolNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetProtocolStackNameSelection((char*)g_deviceName.c_str(), lStartOfSelection, pProtocolNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetProtocolStackNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("protocol stack name = %s\n", pProtocolNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pProtocolNameSel;

	return lResult;
}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
void motor_cmd::cmdReceived(const geometry_msgs::Twist& msg)
{
    ROS_INFO_STREAM("Test: " << msg.linear.x);

    int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;
	long MaxRPM = 7000;
	long valueRight = (msg.linear.x + msg.angular.z)*MaxRPM;
	long valueLeft = (msg.linear.x - msg.angular.z)*MaxRPM;
	long Velocity;
    for (int deviceNum = 1; deviceNum < (NumDevices + 1); deviceNum++)
    {
        if(deviceNum < 4)
        {
            Velocity = valueLeft;
            if(Velocity > MaxRPM)
                Velocity = MaxRPM;
            if(Velocity < -MaxRPM)
                Velocity = -MaxRPM;
        }
        if(deviceNum > 3)
        {
            Velocity = valueRight;
            if(Velocity > MaxRPM)
                Velocity = MaxRPM;
            if(Velocity < -MaxRPM)
                Velocity = -MaxRPM;
        }
        std::cout << "Velocity Left: " << valueLeft << "  " << "Velocity Right: " << valueRight << std::endl;
	    lResult = SetVelocity(g_pKeyHandle, deviceNum, lErrorCode, Velocity);
	    if(lResult != MMC_SUCCESS)
	    {
	    	LogError("SetVelocity", lResult, lErrorCode);
	    }
	}

}

/**
		Finds markers with given ID

		@param dictionary dictionary to be searched
		@param id ID to be detected
    @return List of indices for positions containing the specified id
*/
void motor_cmd::clearFaultCallback(const sensor_msgs::Joy& msg)
{
 int lResult = MMC_FAILED;
 unsigned int ulErrorCode = 0;
	if(msg.buttons[3] == 1)
	{
		ROS_INFO("Clear Faults");
			if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
			{
				LogError("PrepareDemo", lResult, ulErrorCode);
			}
	}
}

////////////////////////////////////////////////////
/*****************CONSTRUCTORS*********************/
////////////////////////////////////////////////////
/**
    Default Constructor
*/
void motor_cmd::motor_cmd(){
  //USB
  g_usNodeId.push_back(2);
  g_pKeyHandle = 0;
  g_deviceName = "EPOS4";
  g_protocolStackName = "MAXON SERIAL V2";
  g_interfaceName = "USB";
  g_portName = "USB0";
  g_baudrate = 1000000;
  NumDevices = 1;

  PrintHeader();
}

/**
		Constructor for specific motor ids and baudrate

		@param ids id number of motors to be used
		@param br baudrate for communications
*/
void motor_cmd(std::vector<int> ids, int br){
  for (int i = 1: i < ids.size(): ++i){
    g_usNodeId.push_back( (unsigned short) ids[i]);
    g_pKeyHandle.push_back(0);
  }
  g_deviceName = "EPOS4";
  g_protocolStackName = "MAXON SERIAL V2";
  g_interfaceName = "USB";
  g_portName = "USB0";
  g_baudrate = br;
  NumDevices = ids.size();

  PrintHeader();
}



////////////////////////////////////////////////////
/***************Print and Commands*****************/
////////////////////////////////////////////////////
/**
		Prints terminal command options
*/
void motor_cmd::PrintUsage()
{
	ROS_INFO("Usage: HelloEposCmd");
	ROS_INFO("-h   : this help");
	ROS_INFO("-n   : node id (default 1)");
	ROS_INFO("-d   : device name (EPOS2, EPOS4, default - EPOS4)" );
	ROS_INFO("-s   : protocol stack name (MAXON_RS232, CANopen, MAXON SERIAL V2, default - MAXON SERIAL V2)" );
	ROS_INFO("-i   : interface name (RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default - USB)" );
	ROS_INFO("-p   : port name (COM1, USB0, CAN0,... default - USB0)");
	ROS_INFO("-b   : baudrate (115200, 1000000,... default - 1000000)");
	ROS_INFO("-l   : list available interfaces (valid device name and protocol stack required)");
	ROS_INFO("-r   : list supported protocols (valid device name required)");
	ROS_INFO("-v   : display device version" << endl;
}

/**
		Logs motor controller error

		@param functionName name of function calling error
		@param p_lResult Error signifier (1=failure, 0= success)
    @param p_ulErrorCode Failure codeed id
*/
void motor_cmd::LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	std::cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

/**
		Makes a divider line
*/
void motor_cmd::SeparatorLine()
{
	const int lineLength = 65;
	for(int i=0; i<lineLength; i++)
	{
		std::cout << "-";
	}
	std::cout << std::endl;
}

/**
		Prints Maxon header
*/
void motor_cmd::PrintHeader()
{
	SeparatorLine();

	ROS_INFO("Epos Command Library--Fast Traverse Program");

	SeparatorLine();
}

/**
		Prints device and port settings
*/
void motor_cmd::PrintSettings()
{
	std::stringstream msg;

	msg << "default settings:" << std::endl;
	msg << "node id             = " << g_usNodeId << std::endl;
	msg << "device name         = '" << g_deviceName << "'" << std::endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << std::endl;
	msg << "interface name      = '" << g_interfaceName << "'" << std::endl;
	msg << "port name           = '" << g_portName << "'"<< std::endl;
	msg << "baudrate            = " << g_baudrate;

	ROS_INFO(msg.str());
	SeparatorLine();
}

/**
		Modifies settings based on input commands

		@param argc number of arguments
		@param argv list of arguments
    @return Success(0)/Failure(1) of command
*/
int motor_cmd::ParseArguments(int argc, char** argv)
{
	int lOption;
	int lResult = MMC_SUCCESS;

	opterr = 0;

	while((lOption = getopt(argc, argv, "hlrvd:s:i:p:b:n:")) != -1)
	{
		switch (lOption)
		{
			case 'h':
				PrintUsage();
				lResult = 1;
				break;
			case 'd':
				g_deviceName = optarg;
				break;
			case 's':
				g_protocolStackName = optarg;
				break;
			case 'i':
				g_interfaceName = optarg;
				break;
			case 'p':
				g_portName = optarg;
				break;
			case 'b':
				g_baudrate = atoi(optarg);
				break;
			case 'n':
				g_usNodeId = (unsigned short)atoi(optarg);
				break;
			case 'l':
				g_eAppMode = AM_INTERFACE_LIST;
				break;
			case 'r':
				g_eAppMode = AM_PROTOCOL_LIST;
				break;
			case 'v':
				g_eAppMode = AM_VERSION_INFO;
				break;
			case '?':  // unknown option...
				stringstream msg;
				msg << "Unknown option: '" << char(optopt) << "'!";
				ROS_INFO(msg.str());
				PrintUsage();
				lResult = MMC_FAILED;
				break;
		}
	}

	return lResult;
}
