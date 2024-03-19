#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>


struct BallInfo {
	int BallXPosition;
	int BallYPosition;

	int BallXVelocity;
	int BallYVelocity;
};


class ImagePorcessor
{
public:
	void initCam(int camId);
	void updatePos();

	ImagePorcessor(cv::Mat& frame) : frame{ frame } {};

	BallInfo getBallInfo() const;

private:
	bool getFrame();
	cv::Point getPosBall();

	cv::VideoCapture cap;

	cv::Mat& frame;

	cv::Point posBall;
	cv::Point velocityBall;

	float WidthCam = 0;
	float HeightCam = 0;

	const int targetHeight = 30;
	const int targetWidth = 60;

	std::string window_name = "Camera Feed";

	std::chrono::time_point<std::chrono::system_clock> frameTime = std::chrono::system_clock::now();



};

