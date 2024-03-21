#pragma once

#include <SFML/Graphics.hpp>

#include <memory>
#include <vector>
#include <array>

#include "JointControl.h"
#include "ImageProcessor.h"
#include "Utils.h"
#include "SerialPort.h"


class SFMLWindow
{
public:
	SFMLWindow(int width, int height, std::shared_ptr<Robot> planarRobot,
		bool enableComArduino, bool enableCamera, bool cameraCalibration,
		bool automaticPlay, int cameraId, std::string portRobot, std::string portArduino);

	~SFMLWindow() {}

	void loop();
	void cornerSetting();

private:

	void initShapes();
	std::array<float, 8> getPosEE();

	void updateXpos(float change);
	void punch();

	// window //
	const int width;
	const int height;
	void manageSFMLevent(sf::RenderWindow& window, sf::Event event);
	void render(sf::RenderWindow& window);

	// camera //
	const bool cameraCalibration;
	const bool enableCamera;

	cv::Mat frame;
	const std::string windowName;
	ImagePorcessor imageProcessor{ frame, windowName };

	// planar robot //
	const bool automaticPlay;
	const std::string robotPort;
	std::shared_ptr<Robot> planarRobot = std::make_shared<Robot>();
	void moveRobotUsingMouse(sf::RenderWindow& window);

	// serial //
	const std::string port;
	Serial serialPort{port.c_str()};
	const bool comArduino;
	void manageArduinoMsgs();


	/// gameObjects ///

	// pingpong ball
	float ballWidth = 10.0f;
	sf::CircleShape pingpongBall{ ballWidth };

	// world variable
	const sf::Vector2f posBaseRobot{ 200, 400 };

	// planar robot variables
	float XpositionEndEffector = 0;

	std::array<float, 2> posEE{};
	std::vector<float> angleMotors{ 0.f, 90.0f, 0.f, 0.f };
	std::vector<float> lengthLinks{ 5.f, 7.75f, 3.f };



};

