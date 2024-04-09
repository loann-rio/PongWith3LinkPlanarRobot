#include "JointControl.h"

void Robot::initRobot()
{
	std::cout << "=== Initialization of the Dynamixel Motor communication ====" << std::endl;
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
	std::cout << "=== Dynamixel Motor communication closed ====" << std::endl;
}

void Robot::moveByJoint(std::vector<float> jointVal)
{
	jointVal[3] *= -1; // the 3rd axis is inverted becose I puted the motor the wrong way
	if (enableCom) _oDxlHandler.sendTargetJointPosition(jointVal);
}

std::vector<float> Robot::cartesianMove(std::vector<float> worldPos, std::vector<float> lengthLinks)
{

	std::vector<float> qi = computeInverseKinematics(worldPos[0], worldPos[1], lengthLinks[0], lengthLinks[1]);

	if (qi.size() >= 3) // ==> qi size == 3 mean we have one or two possible answer
	{
		std::vector<float> vTargetJointPosition;
		vTargetJointPosition.push_back(qi[1]);
		vTargetJointPosition.push_back(deg2rad(90));
		vTargetJointPosition.push_back(qi[2]);
		vTargetJointPosition.push_back(- qi[1] - qi[2]);

		return vTargetJointPosition;
	}

	return {};
}

/// <summary>
/// change the speed of the motors
/// </summary>
/// <param name="speed">speed between 0 and 1024</param>
/// <returns>bool changed worked</returns>
bool Robot::changeSpeed(int speed)
{
	if (!enableCom) return false;
	return _oDxlHandler.setSpeed(speed);
}
