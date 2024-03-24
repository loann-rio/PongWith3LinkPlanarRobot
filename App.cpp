#include "App.h"


void App::run()
{
    if (enableComRobot) planarRobot->initRobot();
    if (enableCamera) imageProcessor.initCam(cameraId);

    SFMLWindow window{ 800, 800, planarRobot,
            enableComArduino, enableCamera, enableCalibration, 
            autoPlay, cameraId, portRobot, portArduino};


    window.loop();

    if (enableComRobot) planarRobot->closeRobot();
}