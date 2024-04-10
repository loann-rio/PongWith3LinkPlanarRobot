#pragma once

#include "SFMLWindow.h"
#include "JointControl.h"

#define enableComRobot false
#define enableComArduino false
#define enableCamera false
#define enableCalibration false
#define autoPlay false
#define cameraId 0
#define portRobot "COM6"
#define portArduino "COM7"


class App
{
public:
	void run();

private:

	// planar robot //
	const std::string robotPort;
	std::shared_ptr<Robot> planarRobot = std::make_shared<Robot>(enableComRobot, portRobot);

};

