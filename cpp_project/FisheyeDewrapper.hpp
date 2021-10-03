#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>


/* TODO: Constructor, */

class FisheyeDewrapper
{
private:
	const double PI = 3.1416;

	float xFov; // output image fov
	float yFov; // or 16:9 equivalent
	cv::Size newSize; // output image size
	/* RPY angles for the world point rotator*/
	float yaw;
	float pitch;
	float roll;

private:
	double polynom[4];		// = { 350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9) };  // Sarcamuzza model 
	cv::Mat map1;   // x map
	cv::Mat map2;   // y map
	std::vector<cv::Point> frameBorder; //border to draw on original image 

private:
	void createMaps();
	cv::Mat rotatePoints(cv::Mat worldPoints);

public:
	FisheyeDewrapper();
	void setSize(int width, int height);
	void setSize(cv::Size size);
	void setFov(float x, float y);
	void setFovWide(float wFov);
	void setCoefficents(double coeffs[4]);
	void setCoefficents(double a1, double a2, double a3, double a4);
	void setRpy(float yaw, float pitch, float roll);

public:
	cv::Point2f projectWorldToFisheye(cv::Mat worldPoint, cv::Size fisheyeSize);
	cv::Point2f projectWorldToPinhole(cv::Mat cameraCoords, cv::Size imgSize);
	cv::Mat projectFisheyeToWorld(cv::Point pixel);
	cv::Mat projectPinholeToWorld(cv::Point pixel, cv::Size imgSize);


	void fillMaps(cv::Size origSize);
	cv::Mat dewrapImage(cv::Mat inputImage);
};


