#include "App.h"

#include "Utils.h"

// std library
#include <algorithm>  
#include <iostream>
#include <vector>
#include <string>
#include <array>


void App::run()
{
    if (enableCom)
        planarRobot->initRobot();


    SFMLWindow window{ 400, 600, planarRobot };
    window.loop();


    if (enableCom)
        planarRobot->closeRobot();


}