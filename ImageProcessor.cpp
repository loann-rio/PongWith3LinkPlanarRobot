#include "ImageProcessor.h"

BallInfo ImagePorcessor::getBallInfo() const
{
	return { posBall.x, posBall.y, velocityBall.x, velocityBall.y };
}

bool ImagePorcessor::showFrame()
{
	cv::imshow("window_name", frame);
	//std::cout << frame[0] << '\n';

	if (cv::waitKey(10) == 27)
	{
		//cartControl.closeRobot();
		std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
		return true;
	}

	return false;
}

void ImagePorcessor::testCam()
{
	while (true) {
		if (!getFrame())
		{
			std::cout << "fail to aquire frame\n";
		}

		cv::imshow("window_name", frame);


		if (cv::waitKey(10) == 27)
		{
			//cartControl.closeRobot();
			std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
			break;
		}

	}
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
		int ptX = x;
		int ptY = y;
	}
}

void  ImagePorcessor::CornerSelection() {

	std::cout << "hellooooooooooooooooooooooooooooooooooooooooooooo\n";
	int ptX = 0;
	int ptY = 0;
	double ptcoord[4][2];
	// Read image from file 
	getFrame();
	
	//set the callback function for any mouse event
	cv::setMouseCallback("window_name", CallBackFunc, NULL);

	//show the image
	showFrame();

	for (int i = 0; i < 4; ++i) {
		cv::waitKey(0);
		ptcoord[i][0] = ptX;
		ptcoord[i][1] = ptY;
		
	}

	// Wait until user press some key
}

void ImagePorcessor::calibration()
{
	// Define the size of the checkerboard
	cv::Size boardSize(5, 8); // Change this according to your checkerboard size

	// Create vectors to store the detected corner points in the images
	std::vector<std::vector<cv::Point3f>> objectPoints;
	std::vector<std::vector<cv::Point2f>> imagePoints;

	// Prepare the object points, which represent the corners of the checkerboard in 3D space
	std::vector<cv::Point3f> obj;
	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {
			obj.push_back(cv::Point3f(j, i, 0));
		}
	}

	bool done = false;
	while (!done) {
		
		/// get frame ///

		getFrame();

		// Convert frame to grayscale
		cv::Mat gray;
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		/// find chessboard ///

		// Find corners in the frame
		std::vector<cv::Point2f> corners;
		if (cv::findChessboardCorners(gray, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE)) {

			// Refine corner locations
			cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			imagePoints.push_back(corners);
			objectPoints.push_back(obj);

			// Draw corners on the frame (optional)
			cv::drawChessboardCorners(frame, boardSize, corners, true);
		}

		done = showFrame();

		// Check if the window was destroyed
		/*if (cv::getWindowProperty("Live Calibration", cv::WND_PROP_AUTOSIZE) == -1) {
			std::cout << "leaving!" << std::endl;
			break;
		}*/
	}

	std::cout << "left!" << std::endl;

	// Calibrate camera
	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;
	double rms = cv::calibrateCamera(objectPoints, imagePoints, frame.size(), cameraMatrix, distCoeffs, rvecs, tvecs);
	std::cout << "test\n";
	// Undistort video
	cap.set(cv::CAP_PROP_FRAME_WIDTH, frame.cols);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame.rows);

	calibrated = true;

	std::cout << "done\n";
}



void ImagePorcessor::initCam(int camId)
{
	cap = cv::VideoCapture{ camId };

	if (cap.isOpened() == false)
	{
		std::cout << "Cannot open the video camera\n";
		std::cin.get(); //wait for any key press
		return;
	}

	WidthCam = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	HeightCam = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

	//cv::namedWindow(window_name); //create a window called "My Camera Feed"

	std::cout << "camOpen with Height = " << HeightCam << "and width = " << WidthCam << "\n";
}


void ImagePorcessor::updatePos()
{
	if (!getFrame())
	{
		std::cout << "fail to aquire frame\n";
	}



	cv::Point point = getPosBall();

	std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - frameTime;
	frameTime = std::chrono::system_clock::now();

	velocityBall = (posBall - point) / (elapsed_seconds.count()); // in pixel per sec
	posBall = point;

	// Draw a red circle at the position of the most orange pixel
	cv::circle(frame, cv::Point{ (point.x * (int)WidthCam / targetWidth), (point.y * (int)HeightCam / targetHeight) }, 10, cv::Scalar(0, 0, 255), -1);

	//show the frame in the created window
	//cv::imshow(window_name, frame);

}

bool ImagePorcessor::getFrame()
{

	bool bSuccess = cap.read(frame); // read a new frame from video 

	//Breaking the while loop if the frames cannot be captured
	if (bSuccess == false)
	{
		std::cout << "Video camera is disconnected" << std::endl;
		std::cin.get(); //Wait for any key press
	}

	if (calibrated)
		cv::undistort(frame, frame, cameraMatrix, distCoeffs);

	//frame = cv::imread("C:\\Users\\loann\\Desktop\\pingPongBall.jpg");
	return bSuccess;
}

cv::Point ImagePorcessor::getPosBall()
{
	// resizee window
	cv::Mat resizedFrame;

	cv::resize(frame, resizedFrame, cv::Size(targetWidth, targetHeight), cv::INTER_LINEAR);

	cvtColor(resizedFrame, resizedFrame, cv::COLOR_BGR2HSV);

	// Define the range of orange color in HSV
	cv::Scalar lowerOrange = cv::Scalar(0, 198, 100);
	cv::Scalar upperOrange = cv::Scalar(22, 255, 255);

	// Threshold the image to get a binary mask
	cv::Mat mask;
	cv::inRange(resizedFrame, lowerOrange, upperOrange, mask);

	// Find contours in the binary mask
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// Find the contour with the largest area (most orange)
	double maxArea = 0;
	int maxAreaIndex = -1;
	for (size_t i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i]);
		if (area > maxArea) {
			maxArea = area;
			maxAreaIndex = i;
		}
	}

	// Get the centroid of the largest contour
	if (maxAreaIndex != -1) {
		cv::Moments mu = moments(contours[maxAreaIndex]);
		cv::Point centroid(mu.m10 / mu.m00 + .5f, mu.m01 / mu.m00 + .5f);

		//std::cout << "Position of the most orange pixel: " << centroid << std::endl;

		return centroid;
	}
	else {
		std::cout << "No orange pixels found in the image." << std::endl;

		return cv::Point{ -1 };
	}
}
