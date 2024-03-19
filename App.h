#pragma once

#include "SFMLWindow.h"
#include "JointControl.h"

class App
{
public:
	void run();

private:
	std::shared_ptr<Robot> planarRobot = std::make_shared<Robot>();
};

