#pragma once

#include "SFMLWindow.h"
#include "JointControl.h"

#define enableComRobot false
#define enableComArduino false
#define enableCamera false
#define enableCalibration false
#define autoPlay false
#define cameraId 2
#define portRobot "COM1"
#define portArduino "COM2"


class App
{
public:
	void run();

private:

	std::shared_ptr<Robot> planarRobot = std::make_shared<Robot>(enableComRobot);

};

