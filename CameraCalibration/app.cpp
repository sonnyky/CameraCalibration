#include "app.h"
#include "util.h"
#include <thread>
#include <chrono>

#include <numeric>
// oscpack
#include <ip/UdpSocket.h>
#include <osc/OscOutboundPacketStream.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


using namespace std;

// Constructor
Capture::Capture()
{
	// Initialize
	initialize();
}

// Destructor
Capture::~Capture()
{
	// Finalize
	finalize();
}

void Capture::initialize() {
	
	cvNamedWindow("Color", CV_WINDOW_NORMAL);

	initializeSensor();
	initializeColorImage();
	cv::setMouseCallback("Color", mouseCallback, this);

	setupCalibration();
	//imwrite("../ReferenceImages/arucoMarker849.jpg", markerImage);
	
}

void Capture::finalize() {

}

void Capture::run()
{
	while (true) {
		update();
		draw();
		show();
        if(waitKey(30) >= 0) break;
	}
}

void Capture::filterColor(Mat image) 
{
	int largest_area = 0;
	int largest_contour_index = 0;

	cv::Scalar   min(90, 190, 80);
	cv::Scalar   max(170, 255, 255);
	cv::Mat mask, mask1, mask2, mask_grey;
	cv::Mat hsv_image;
	cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
	mask1 = image.clone(); mask2 = image.clone();
	//filter the image in BGR color space
	cv::inRange(hsv_image, min, max, mask);
	//cv::inRange(hsv_image, cv::Scalar(55, 100, 50), cv::Scalar(65, 255, 255), mask1);
	//cv::inRange(hsv_image, cv::Scalar(50, 100, 50), cv::Scalar(70, 255, 255), mask2);
	std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
	std::vector<cv::Vec4i> hierarchy;
	//mask = mask1 | mask2;
	findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
	vector<Rect> bounding_rect(contours.size());

	for (int i = 0; i< contours.size(); i++) // iterate through each contour. 
	{
		double a = contourArea(contours[i], false);  //  Find the area of contour
	/*
	if (a>largest_area) {
	largest_area = a;
	largest_contour_index = i;                //Store the index of largest contour
	bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
	}
	*/

		bounding_rect.push_back(boundingRect(contours[i]));

	}
	cv::Scalar color(255, 255, 255);
	drawContours(mask, contours, largest_contour_index, color, CV_FILLED, 8, hierarchy); 
	cv::imshow("filtered", mask);	
}

// Initialize Sensor
inline void Capture::initializeSensor()
{
	// Open Sensor
	ERROR_CHECK(GetDefaultKinectSensor(&kinect));

	ERROR_CHECK(kinect->Open());

	// Check Open
	BOOLEAN isOpen = FALSE;
	ERROR_CHECK(kinect->get_IsOpen(&isOpen));
	if (!isOpen) {
		throw std::runtime_error("failed IKinectSensor::get_IsOpen( &isOpen )");
	}
}

// Initialize color image
inline void Capture::initializeColorImage()
{
	// Open Color Reader
	ComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

	// Retrieve Color Description
	ComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK(colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
	ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth)); // 1920
	ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight)); // 1080
	ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel)); // 4

	// Allocation of Color Buffer
	colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
}

// Update Data
void Capture::update()
{
	// Update Color
	updateColor();
}

// Update Color
inline void Capture::updateColor()
{
	// Retrieve Color Frame
	ComPtr<IColorFrame> colorFrame;
	const HRESULT ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (FAILED(ret)) {
		return;
	}

	// Convert Format ( YUY2 -> BGRA )
	ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));
}

inline void Capture::calibrate(Mat image) {

	if ((clock() - startClockTick) / (double)CLOCKS_PER_SEC < timeDelayBeforeCalibration) {
		return;
	}
	// Convert to Grayscale

	vector<Point2f> pointBuf;
	int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
	Mat gray;
	cv::cvtColor(image, gray, CV_BGR2GRAY);

	vector< Point2f > corners;
	bool found = false;

	found = cv::findChessboardCorners(image, boardSize, corners,
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	if (found)
	{
		cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		drawChessboardCorners(gray, boardSize, corners, found);
	}

}

// Draw Data
void Capture::draw()
{

	// Draw Color
	drawColor();

}

// Draw Color
inline void Capture::drawColor()
{
	// Convert Coordinate Buffer to cv::Mat
	//Mapping color and depth frame taken from https://gist.github.com/UnaNancyOwen/7e2c685752e16f8e42cc#file-main-cpp-L232
	colorMat = Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]).clone();
	cv::flip(colorMat, colorMat, 1);
}

// Show Data
void Capture::show()
{
	// Show Color
	showColor();

}

// Show Color
inline void Capture::showColor()
{
	if (colorMat.empty()) {
		return;
	}
	// Calibrate next image sample
	calibrate(colorMat);
	cv::imshow("Color", colorMat);
}



void Capture::mouseCallback(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		Capture* self = static_cast<Capture*>(userdata);
		self->doMouseCallback(event, x, y, flags);
	}
}

void Capture::doMouseCallback(int event, int x, int y, int flags) {
	if (flags == (cv::EVENT_FLAG_LBUTTON))
	{
		std::cout << "Left mouse clicked" << std::endl;
	}

	if (flags == (cv::EVENT_FLAG_LBUTTON + cv::EVENT_FLAG_SHIFTKEY))
	{
		
		std::cout << "Shift+Left click" << std::endl;
	}

}

void Capture::setupCalibration() {
	numOfCornersHorizontal = 9;
	numOfCornersVertical = 6;
	numOfCalibrationLoop = 30;
	boardSize = Size(numOfCornersHorizontal, numOfCornersVertical);
	int board_n = numOfCornersHorizontal * numOfCornersVertical;
	timeDelayBeforeCalibration = 2;
	startClockTick = clock();
}