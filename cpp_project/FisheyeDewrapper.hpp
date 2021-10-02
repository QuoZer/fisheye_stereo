#include "opencv2/core.hpp"
#include <vector>

const double PI = 3.1416;

/* TODO: Constructor, */

class FisheyeDewrapper
{
private:
	int xFov = 90; // output image fov
	int yFov = 90; // or 16:9 equivalent
	cv::Size newSize; // output image size
private:
	double polynom[4] = { 350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9) };  // Sarcamuzza model 
	cv::Mat map1;   // x map
	cv::Mat map2;   // y map

public:
	FisheyeDewrapper FisheyeDewrapper();
	void setSize(int width, int height);
	void setSize(cv::Size size);
	void setFov(int x, int y);
	void setFov(int x);
public:
	cv::Point projectWorldToFisheye(cv::Mat worldPoint, cv::Size fisheyeSize);
	cv::Point2f projectWorldToPinhole(cv::Mat cameraCoords, cv::Size imgSize);
	cv::Mat projectFisheyeToWorld(cv::Point pixel);
	cv::Mat projectPinholeToWorld(cv::Point pixel, cv::Size imgSize, int fov);

	cv::Mat rotatePoints(cv::Mat worldPoints, float yaw, float pitch, float roll);
	void fillMaps(cv::Mat& map1, cv::Mat& map2, cv::Size origSize, cv::Size newSize, int fov, std::vector<cv::Point>& frameBorder);
};


