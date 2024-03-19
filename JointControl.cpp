#include "JointControl.h"

void Robot::initRobot()
{
	std::cout << "===Initialization of the Dynamixel Motor communication====" << std::endl;
	_oDxlHandler.setDeviceName(_robotDxlPortName);
	_oDxlHandler.setProtocolVersion(_robotDxlProtocol);
	_oDxlHandler.openPort();
	_oDxlHandler.setBaudRate(_robotDxlBaudRate);
	_oDxlHandler.enableTorque(true);
	_oDxlHandler.setSpeed(64);
}

void Robot::closeRobot()
{
	_oDxlHandler.enableTorque(false);
	_oDxlHandler.closePort();
}

void Robot::moveByJoint(std::vector<float> jointVal)
{


	_oDxlHandler.sendTargetJointPosition(jointVal);
	//std::cout << "command sent\n";
}

std::vector<float> Robot::cartesianMove(std::vector<float> worldPos, std::vector<float> lengthLinks)
{

	std::vector<float> qi = computeInverseKinematics(worldPos[0], worldPos[1], lengthLinks[0], lengthLinks[1]);

	if (qi.size() >= 3)
	{

		std::vector<float> vTargetJointPosition;
		vTargetJointPosition.push_back(qi[1]);
		vTargetJointPosition.push_back(deg2rad(90));
		vTargetJointPosition.push_back(qi[2]);
		vTargetJointPosition.push_back(- qi[1] - qi[2]);

		//_oDxlHandler.sendTargetJointPosition(vTargetJointPosition);

		return vTargetJointPosition;
	
	}

	return {};
}

bool Robot::changeSpeed(int speed)
{
	return _oDxlHandler.setSpeed(speed);
}
