#define _USE_MATH_DEFINES
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include <numbers>
#include <math.h>

#define SCARAMUZZA		10
#define ATAN			20
#define REV_SCARAMUZZA	30

/* TODO: -Constructor, 
		 -
*/

class FisheyeDewarper
{
private:	/* Parameters */
	const double PI = M_PI; 
	float xFov;							// output image fov
	float yFov;							// or 16:9 equivalent
	cv::Size oldSize;					// input image size
	cv::Size newSize;					// output image size
	/* RPY angles for the world point rotator*/
	float yaw;
	float pitch;
	float roll;

private:	/* Data */
	double errorsum;
	/* Intrinsics */
	std::vector <double> polynom;		// Scaramuzza model coefficients
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 
	/* Structures */
	cv::Mat map1;						// x map
	cv::Mat map2;						// y map
	std::vector<cv::Point> frameBorder; //border to draw on the original image 

private:	/* Internal functions */
	void createMaps();
	/* Transformations */
	void toCenter(cv::Point& cornerPixel, cv::Size imagesize);
	void toCorner(cv::Point& centerPixel, cv::Size imagesize);
	cv::Mat rotatePoint(cv::Mat worldPoint);
	/* Projection functions */
	cv::Point2f projectWorldToFisheyeAtan(cv::Mat worldPoint);
	cv::Point2d projectWorldToFisheye(cv::Mat worldPoint);
	cv::Point2f projectWorldToPinhole(cv::Mat cameraCoords);
	cv::Mat projectFisheyeToWorld(cv::Point pixel);
	cv::Mat projectPinholeToWorld(cv::Point pixel);
	cv::Point reverseScaramuzza(cv::Point pixel);	// Fisheye to Pinhole 
	/* Tools */
	void fillMapsScaramuzza();
	void fillMapsRevScaramuzza();
	void fillMapsAtan();
	void setFovWide(float wFov);

public:		/* Settings */
	FisheyeDewarper();
	void setSize(int oldWidth, int oldHeight, int newWidth, int  newHeight, float wideFov);
	void setSize(cv::Size oldsize, cv::Size newsize, float wideFov);
	void setIntrinsics(double coeffs[4], cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor);
	void setIntrinsics(double a1, double a2, double a3, double a4, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor);
	void setRpy(float yaw, float pitch, float roll);

	std::vector<cv::Point> getBorder();

public:		/*  */
	void fillMaps(int mode);
	cv::Mat dewrapImage(cv::Mat inputImage);
	
};


