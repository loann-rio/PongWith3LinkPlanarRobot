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
	
	void initRobot();
	void closeRobot();

	void moveByJoint(std::vector<float> jointVal);
	std::vector<float> cartesianMove(std::vector<float> worldPos, std::vector<float> lengthLinks);

	bool changeSpeed(int speed);

private:

	// communication variables
	DynamixelHandler _oDxlHandler;
	const std::string _robotDxlPortName = "COM6";
	const float _robotDxlProtocol = 2.0;
	const int _robotDxlBaudRate = 1000000;


};

