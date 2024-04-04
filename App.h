#pragma once

#include "SFMLWindow.h"
#include "JointControl.h"

#define enableComRobot true
#define enableComArduino false
#define enableCamera true
#define enableCalibration true
#define autoPlay false
#define cameraId 0
#define portRobot "COM6"
#define portArduino "COM2"


class App
{
public:
	void run();

private:



	// camera //
	/*cv::Mat frame;
	const std::string windowName;
	ImagePorcessor imageProcessor{ frame, windowName };*/

	// planar robot //
	const std::string robotPort;
	std::shared_ptr<Robot> planarRobot = std::make_shared<Robot>(enableComRobot);
	///void moveRobotUsingMouse(sf::RenderWindow& window);

	// serial //
	/*const std::string port;
	Serial serialPort{ port.c_str() };
	const bool comArduino;*/
	//void manageArduinoMsgs();

};

