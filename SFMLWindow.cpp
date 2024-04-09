#include "SFMLWindow.h"

#include <conio.h> 
#include <stdlib.h> 
#include <chrono>
#include <thread>

#define cm2Pix 10
#define widthLink 10
#define keyStep 1.f

SFMLWindow::SFMLWindow(int width, int height, std::shared_ptr<Robot> planarRobot, bool enableComArduino, bool enableCamera, bool cameraCalibration, bool automaticPlay, int cameraId, std::string portRobot, std::string portArduino) : 
    width{width}, height{height}, planarRobot{ planarRobot }, comArduino { enableComArduino }, enableCamera { enableCamera }, cameraCalibration { cameraCalibration }, automaticPlay{ automaticPlay }, robotPort{ portRobot }, port{ portArduino }
{
	initShapes();

    if (enableCamera) {
        imageProcessor.initCam(cameraId);
    }
        
}

void SFMLWindow::loop()
{

    // user corner setting
    if (cameraCalibration)
        cornerSetting();

    // create the window
    sf::RenderWindow window(sf::VideoMode(width, height), "2 link planar robot pong");
    
    while (window.isOpen())
    {
        
        // manage sfml event
        sf::Event event;
        while (window.pollEvent(event))  {   
            manageSFMLevent(window, event);
        }

        // manage arduino event
        if (comArduino) {
            manageArduinoMsgs();
        }

        // manage image input
        if (enableCamera) {
            imageProcessor.updatePos();
            imageProcessor.showFrame();
            //auto frameInfo = imageProcessor.getBallInfo();

            //std::cout << frameInfo.BallXPosition << " " << frameInfo.BallYPosition << "\n";
        }

        if (automaticPlay) {
            auto frameInfo = imageProcessor.getBallInfo();
            std::vector<float> distances{};

            /*float distCorners = sqrt(pow(cornerPos[0].x - cornerPos[1].x, 2) + pow(cornerPos[0].y - cornerPos[1].y, 2));

            for (const sf::Vector2f& corner : cornerPos) {
                distances.push_back( 
                    sqrt(pow(corner.x - frameInfo.BallXPosition, 2) + pow(corner.y - frameInfo.BallYPosition, 2)) / distCorners
                );
            }*/

            float x1 = (cornerPos[0].x - frameInfo.BallXPosition*10.666f) / (cornerPos[0].x - cornerPos[1].x);
            float x2 = (cornerPos[3].x - frameInfo.BallXPosition*10.666f) / (cornerPos[3].x - cornerPos[2].x);

            float y1 = (cornerPos[0].y - frameInfo.BallYPosition*16) / (cornerPos[0].y - cornerPos[3].y);
            float y2 = (cornerPos[1].y - frameInfo.BallYPosition*16) / (cornerPos[1].y - cornerPos[2].y);
            sf::Vector2f realPos = { (x1 + x2) / 2, (y1 + y2) / 2};
            pingpongBall.setPosition(rectangle.getPosition() + sf::Vector2f({realPos.x * 600, realPos.y * 300}));

            std::cout << (1 - realPos.y) * 28 - 14.f << "\n";

            std::vector<float> r = planarRobot->cartesianMove({ 6.f , (1 - realPos.y) * 25 - 12.5f }, { 5.f, 8.25f, 3.f });

            if (!r.empty())
                angleMotors = r;

            if (realPos.x < 0.1f) {
                //punch();
            }
          
        }


        moveRobotUsingMouse(window);

        planarRobot->moveByJoint(angleMotors);

        render(window); // -> render the window  #impotant info #useful
        
    }
}

std::vector<sf::Vector2f> SFMLWindow::cornerSetting()
{
    sf::Image image;
    cv::Mat frame;

    while (frame.empty()) {
        frame = imageProcessor.getNewFrame();
    }

    cv::Mat newFrame;

    cv::cvtColor(frame, newFrame, cv::COLOR_BGR2RGBA);

    image.create(newFrame.cols, newFrame.rows, newFrame.ptr());
    sf::Texture texture;
    texture.loadFromImage(image);  //Load Texture from image
    sf::Sprite sprite;
    sprite.setTexture(texture);

    sf::RenderWindow window(sf::VideoMode(frame.cols, frame.rows), "2 link planar robot pong");


    std::vector<sf::CircleShape> corners;
    cornerPos.clear();


    bool clicking = false;
 

    while (window.isOpen())
    {

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed || sf::Keyboard::isKeyPressed(sf::Keyboard::Escape) || cornerPos.size() == 4)
                window.close();

            if (event.mouseButton.button == sf::Mouse::Left) 
            {
                if (!clicking)
                {
                    clicking = true;
                    sf::Vector2f posMouseToRob = sf::Vector2f{ sf::Mouse::getPosition(window) };
                    cornerPos.push_back(posMouseToRob);

                    std::cout << posMouseToRob.x  << " "  << posMouseToRob.y << "\n";
                    sf::CircleShape corner{ 5.f };
                    corner.setFillColor(sf::Color::Blue);
                    corner.setPosition(posMouseToRob);
                    corners.push_back(corner);
                }
            }
            else 
            {
                clicking = false;
            }
           
        }

        window.clear({ 255, 255, 255, 255 });
        window.draw(sprite);
        for (const sf::CircleShape& corner : corners)
            window.draw(corner);
        window.display();

    }

    return cornerPos;
}

void SFMLWindow::updateXpos(float change)
{
    XpositionEndEffector = std::min(std::max(-20.f, XpositionEndEffector + change), 20.f);
    //std::cout << XpositionEndEffector << "\n";
    //angleMotors = { std::min(pi<float> / 2.f, std::max(XpositionEndEffector, -pi<float> / 2.f)), pi<float> / 2.f, -1 * XpositionEndEffector / 2 , -1 * XpositionEndEffector / 2 };
    std::vector<float> r = planarRobot->cartesianMove({ 6.5f, XpositionEndEffector }, { 5.f, 8.25f, 3.f });

    if (!r.empty())
        angleMotors = r;
}

void SFMLWindow::initShapes()
{
    pingpongBall.setFillColor(sf::Color::Green);

    rectangle.setPosition(100.f, 100.f); // Position of the top-left corner of the rectangle
    rectangle.setFillColor(sf::Color::Transparent); // Fill color of the rectangle
    rectangle.setOutlineThickness(2.f); // Outline thickness
    rectangle.setOutlineColor(sf::Color::Red); // Outline color

    /*innerCercle.setPosition(posBaseRobot);
    outerCercle.setPosition(posBaseRobot);

    innerCercle.setOutlineColor(sf::Color::Green);
    outerCercle.setOutlineColor(sf::Color::Green);

    innerCercle.setOutlineThickness(2);
    outerCercle.setOutlineThickness(2);*/

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

    std::vector<float> r = planarRobot->cartesianMove({ 6.5f + (float)((sin(acos(XpositionEndEffector / 12.f)) * 12.f - 6.5)*2/3), XpositionEndEffector }, { 5.f, 8.25f, 3.f });

    if (!r.empty())
        angleMotors = r;

    planarRobot->moveByJoint(angleMotors);


    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    r = planarRobot->cartesianMove({ 6.5f, XpositionEndEffector }, { 5.f, 8.25f, 3.f });

    if (!r.empty())
        angleMotors = r;

    planarRobot->moveByJoint(angleMotors);

    planarRobot->changeSpeed(64);
}

void SFMLWindow::manageSFMLevent(sf::RenderWindow& window, sf::Event event)
{
    if (event.type == sf::Event::Closed || sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
        window.close();

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
        updateXpos(keyStep);
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
        updateXpos(-keyStep);
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
        punch();
    }
}

void SFMLWindow::render(sf::RenderWindow& window)
{
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

    window.draw(rectangle);
    window.draw(link1, 4, sf::Quads);
    window.draw(link2, 4, sf::Quads);
    window.draw(link3, 4, sf::Quads);

    window.draw(pingpongBall);
    window.display();

}

sf::Vector2f SFMLWindow::posBall(std::vector<float> distances)
{

    //std::vector<sf::Vector2f> corners{
    //    {0, 0}, {0, 1} , {1, 30}, {60, 0}
    //};
    //
    //// Check if there are enough fixed points and distances
    //if (corners.size() < 3 || corners.size() != distances.size()) {
    //    std::cerr << "Invalid input" << std::endl;
    //    return { 0.0, 0.0 };
    //}
    //
    //// Calculate coefficients for linear equations
    //double a = 2 * (corners[1].x - corners[0].x);
    //double b = 2 * (corners[1].y - corners[0].y);
    //double c = pow(distances[0], 2) - pow(distances[1], 2) - pow(corners[0].x, 2) +
    //    pow(corners[1].x, 2) - pow(corners[0].y, 2) + pow(corners[1].y, 2);
    //double d = 2 * (corners[2].x - corners[1].x);
    //double e = 2 * (corners[2].y - corners[1].y);
    //double f = pow(distances[1], 2) - pow(distances[2], 2) - pow(corners[1].x, 2) +
    //    pow(corners[2].x, 2) - pow(corners[1].y, 2) + pow(corners[2].y, 2);
    //
    //// Solve for x and y
    //double x = (c * e - f * b) / (a * e - b * d);
    //double y = (c * d - a * f) / (b * d - a * e);
    //
    //return sf::Vector2f(x, y);


    return {};


}

void SFMLWindow::moveRobotUsingMouse(sf::RenderWindow& window)
{
    // get pos of the mouse compare to the robot base
    sf::Vector2f posMouseToRob = (sf::Vector2f{ sf::Mouse::getPosition(window) } - posBaseRobot) / (float)cm2Pix;

    // get joint angle to reach the mouse (take only 2 links in account)
    std::vector<float> r = planarRobot->cartesianMove({ posMouseToRob.x-3.f, -posMouseToRob.y }, { 5.f, 8.25f, 3.f });

    if (!r.empty())
        angleMotors = r;
}

void SFMLWindow::manageArduinoMsgs()
{
    std::vector<std::string> msgs = serialPort.getCommandFromSerial();
    for (std::string& msg : msgs) {
        if (msg[0] == 'p') {
            punch();
            continue;
        }

        // j'ai honte de moi:
        if (msg[0] != '-')
            msg = "0" + msg;

        if (stoi(msg) != 0)
            updateXpos((float)stoi(msg) / 70.0f);
    }
}
