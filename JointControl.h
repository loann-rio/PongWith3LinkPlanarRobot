#pragma once


#include <chrono>
#include <thread>
#include <array>
#include <assert.h>

#include "DynamixelHandler.h"
#include "Utils.h"
#include "Kinematics.h"

class Robot {
public:
	Robot(bool enableCom) : enableCom{ enableCom } {}

	void initRobot();
	void closeRobot();

	void moveByJoint(std::vector<float> jointVal);
	std::vector<float> cartesianMove(std::vector<float> worldPos, std::vector<float> lengthLinks);

	bool changeSpeed(int speed);
	std::array<float, 8> getPosJoints();

	void changeEEXposition(float change);

private:

	// communication variables
	DynamixelHandler _oDxlHandler;
	const std::string _robotDxlPortName = "COM6";
	const float _robotDxlProtocol = 2.0;
	const int _robotDxlBaudRate = 1000000;

	const bool enableCom = false;

	std::vector<float> angleMotors{ 0.f, 90.0f, 0.f, 0.f };
	const std::vector<float> lengthLinks{ 5.f, 7.75f, 3.f };
	float XpositionEndEffector = 0;


};

