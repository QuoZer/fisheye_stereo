#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>

#define SARCAMUZZA 10
#define ATAN	   20
#define REV_SARCAMUZZA	30

/* TODO: -Constructor, 
		 -Universal fillMap for different models	*/

class FisheyeDewrapper
{
private:	/* Parameters */
	const double PI = 3.1416; 
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
	std::vector <double> polynom;					// Sarcamuzza model coefficients
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Whatever that is in the stretch matrix
	double lambda;						// Scale factor 
	/* Structures */
	cv::Mat map1;   // x map
	cv::Mat map2;   // y map
	std::vector<cv::Point> frameBorder; //border to draw on original image 

private:	/* Internal functions */
	void createMaps();
	/*  */
	void toCenter(cv::Point& cornerPixel, cv::Size imagesize);
	void toCorner(cv::Point& centerPixel, cv::Size imagesize);
	cv::Mat rotatePoint(cv::Mat worldPoint);
	/* Projection functions */
	cv::Point2f projectWorldToFisheye(cv::Mat worldPoint);
	cv::Point2f projectWorldToPinhole(cv::Mat cameraCoords);
	cv::Mat projectFisheyeToWorld(cv::Point pixel);
	cv::Mat projectPinholeToWorld(cv::Point pixel);
	cv::Point reverseSarcamuzza(cv::Point pixel);	// Fisheye to Pinhole 
	cv::Point2d worldToFisheye(cv::Mat worldPoint);
	/*  */
	void fillMapsSarcamuzza();
	void fillMapsRevSarcamuzza();
	void fillMapsAtan();
	void setFovWide(float wFov);

public:		/* Settings */
	FisheyeDewrapper();
	void setSize(int oldWidth, int oldHeight, int newWidth, int  newHeight, float wideFov);
	void setSize(cv::Size oldsize, cv::Size newsize, float wideFov);
	void setIntrinsics(double coeffs[4], cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor);
	void setIntrinsics(double a1, double a2, double a3, double a4, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor);
	void setRpy(float yaw, float pitch, float roll);

public:		/*  */
	void fillMaps(int mode);
	cv::Mat dewrapImage(cv::Mat inputImage);
	
};


