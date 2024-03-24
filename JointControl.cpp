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
	if (enableCom) _oDxlHandler.sendTargetJointPosition(jointVal);
}

std::vector<float> Robot::cartesianMove(std::vector<float> worldPos, std::vector<float> lengthLinks)
{

	std::vector<float> qi = computeInverseKinematics(worldPos[0], worldPos[1], lengthLinks[0], lengthLinks[1]);

	if (qi.size() >= 3) // ==> qi size ==3 mean we have one or two possible answer
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

bool Robot::changeSpeed(int speed)
{
	if (!enableCom) return false;
	return _oDxlHandler.setSpeed(speed);
}

std::array<float, 8> Robot::getPosJoints()
{
	// first link
	float x1 = cos(angleMotors[0]) * lengthLinks[0];
	float y1 = sin(angleMotors[0]) * lengthLinks[0];

	// second link
	float x2 = x1 + cos(angleMotors[0] + angleMotors[2]) * lengthLinks[1];
	float y2 = y1 + sin(angleMotors[0] + angleMotors[2]) * lengthLinks[1];

	// third link (ee)
	float x3 = x2 + cos(angleMotors[0] + angleMotors[2] + angleMotors[3]) * lengthLinks[2];
	float y3 = y2 + sin(angleMotors[0] + angleMotors[2] + angleMotors[3]) * lengthLinks[2];

	return { 0, 0, x1, y1, x2, y2, x3, y3 };
}

void Robot::changeEEXposition(float change)
{
	XpositionEndEffector = std::min(std::max(-20.f, XpositionEndEffector + change), 20.f);
	std::vector<float> r = this->cartesianMove({ 6.5f, XpositionEndEffector }, { 5.f, 7.75f, 3.f });

	if (!r.empty())
		angleMotors = r;
}
