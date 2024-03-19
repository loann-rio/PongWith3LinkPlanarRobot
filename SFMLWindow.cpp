#include "SFMLWindow.h"

#include <conio.h> 
#include <stdlib.h> 
#include <chrono>
#include <thread>

#define cm2Pix 10
#define widthLink 10

SFMLWindow::SFMLWindow(int width, int height, std::shared_ptr<Robot> planarRobot) : width{width}, height{height}, planarRobot{ planarRobot }
{
	initShapes();
}

void SFMLWindow::initShapes()
{
	pingpongBall.setFillColor(sf::Color::Green);

    innerCercle.setPosition(posBaseRobot);
    outerCercle.setPosition(posBaseRobot);

    innerCercle.setOutlineColor(sf::Color::Green);
    outerCercle.setOutlineColor(sf::Color::Green);

    innerCercle.setOutlineThickness(2);
    outerCercle.setOutlineThickness(2);
    
}

std::array<float, 8> SFMLWindow::getPosEE()
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

    //std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << " " << x3 << " " << y3 << "\n";

    return { 0, 0, x1, y1, x2, y2, x3, y3 };
}

void SFMLWindow::punch()
{
    planarRobot->changeSpeed(255);
    std::vector<float> r = planarRobot->cartesianMove({ 6.5f + (float)((sin(acos(XpositionEndEffector / 12.f)) * 12.f - 6.5) / 2), XpositionEndEffector }, { 5.f, 7.75f, 3.f });


    if (!r.empty())
        angleMotors = r;



    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    r = planarRobot->cartesianMove({ 6.5f, XpositionEndEffector }, { 5.f, 7.75f, 3.f });

    if (!r.empty())
        angleMotors = r;

    planarRobot->changeSpeed(64);
}

void SFMLWindow::loop()
{

    sf::RenderWindow window(sf::VideoMode(width, height), "2 link planar robot pong");
   

    while (window.isOpen())
    {
        
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed || sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
                window.close();

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
                updateXpos(.1f);
            }

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
                updateXpos(-0.1f);
            }

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
                punch();
            }
        }

        // get pos of the mouse compare to the robot base
        sf::Vector2f posMouseToRob = (sf::Vector2f{ sf::Mouse::getPosition(window) } - posBaseRobot) / (float) cm2Pix;

        // get joint angle to reach the mouse (take only 2 links in account)
        std::vector<float> r = planarRobot->cartesianMove({ posMouseToRob.x, -posMouseToRob.y }, { 5.f, 7.75f, 3.f });
        
        if (!r.empty()) 
            angleMotors = r;

        //planarRobot->moveByJoint(angleMotors);


        // display
        sf::Vertex link1[4];
        sf::Vertex link2[4];
        sf::Vertex link3[4];

        {

            std::array<float, 8> posee = getPosEE();

            for (int i = 0; i < 4; i++) {
                posee[i * 2] = posBaseRobot.x + posee[i * 2] * cm2Pix;
                posee[i * 2 + 1] = posBaseRobot.y - posee[i * 2 + 1] * cm2Pix;
            }

            sf::Vector2f point1 = { posee[0], posee[1] };
            sf::Vector2f point2 = { posee[2], posee[3] };
            sf::Vector2f point3 = { posee[4], posee[5] };
            sf::Vector2f point4 = { posee[6], posee[7] };
            float thickness = 10;

            {
                // link 1 //

                sf::Vector2f direction = point2 - point1;
                sf::Vector2f unitDirection = direction / std::sqrt(direction.x * direction.x + direction.y * direction.y);
                sf::Vector2f unitPerpendicular(-unitDirection.y, unitDirection.x);

                sf::Vector2f offset = (thickness / 2.f) * unitPerpendicular;

                link1[0].position = point1 + offset;
                link1[1].position = point2 + offset;
                link1[2].position = point2 - offset;
                link1[3].position = point1 - offset;

                for (int i = 0; i < 4; ++i)
                    link1[i].color = sf::Color::Blue;
            }

            {
                // link 2 //

                sf::Vector2f direction = point3 - point2;
                sf::Vector2f unitDirection = direction / std::sqrt(direction.x * direction.x + direction.y * direction.y);
                sf::Vector2f unitPerpendicular(-unitDirection.y, unitDirection.x);

                sf::Vector2f offset = (thickness / 2.f) * unitPerpendicular;

                link2[0].position = point2 + offset;
                link2[1].position = point3 + offset;
                link2[2].position = point3 - offset;
                link2[3].position = point2 - offset;

                for (int i = 0; i < 4; ++i)
                    link2[i].color = sf::Color::Red;
            }

            {
                // link 3 //

                sf::Vector2f direction = point4 - point3;
                sf::Vector2f unitDirection = direction / std::sqrt(direction.x * direction.x + direction.y * direction.y);
                sf::Vector2f unitPerpendicular(-unitDirection.y, unitDirection.x);

                sf::Vector2f offset = (thickness / 2.f) * unitPerpendicular;

                link3[0].position = point3 + offset;
                link3[1].position = point4 + offset;
                link3[2].position = point4 - offset;
                link3[3].position = point3 - offset;

                for (int i = 0; i < 4; ++i)
                    link3[i].color = sf::Color::Black;

            }

        }

        window.clear({ 255, 255, 255, 255 });

        window.draw(link1, 4, sf::Quads);
        window.draw(link2, 4, sf::Quads);
        window.draw(link3, 4, sf::Quads);

        window.draw(pingpongBall);
        window.display();
    }
}



