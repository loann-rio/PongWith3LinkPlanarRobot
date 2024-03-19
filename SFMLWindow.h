#pragma once

#include <SFML/Graphics.hpp>

#include <memory>
#include <array>

#include "JointControl.h"
#include "Utils.h"

class SFMLWindow
{
public:
	SFMLWindow(int width, int height, std::shared_ptr<Robot> planarRobot);
	~SFMLWindow() {}

	void loop();

private:

	void initShapes();
	std::array<float, 8> getPosEE();


	void updateXpos(float change)
	{
		XpositionEndEffector = std::min(std::max(-20.f, XpositionEndEffector + change), 20.f);
		//angleMotors = { std::min(pi<float> / 2.f, std::max(XpositionEndEffector, -pi<float> / 2.f)), pi<float> / 2.f, -1 * XpositionEndEffector / 2 , -1 * XpositionEndEffector / 2 };
		std::vector<float> r = planarRobot->cartesianMove({ 6.5f, XpositionEndEffector }, { 5.f, 7.75f, 3.f });

		if (!r.empty())
			angleMotors = r;
	}

	void punch();

	// planar robot
	std::shared_ptr<Robot> planarRobot;

	// window
	const int width;
	const int height;


	// gameObjects

	// pingpong ball
	float ballWidth = 10.0f;
	sf::CircleShape pingpongBall{ ballWidth };

	// inner cercle
	float innerCercleWidth = 6.5f * 10;
	sf::CircleShape innerCercle{ innerCercleWidth };

	// inner cercle
	float OuterCercleWidth = 6.5f * 10;
	sf::CircleShape outerCercle{ OuterCercleWidth };

	// world variable
	const sf::Vector2f posBaseRobot{ 200, 400 };

	// planar robot variables
	float XpositionEndEffector = 0;

	std::array<float, 2> posEE{};
	std::vector<float> angleMotors{ 0.f, 90.0f, 0.f, 0.f };
	std::vector<float> lengthLinks{ 5.f, 7.75f, 3.f };



};

