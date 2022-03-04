#include "FisheyeDewarper.hpp"


enum StereoMethod
{
	BM,
	SGBM
};

class Stereopair
{
	cv::StereoMatcher* matcher;

	FisheyeDewarper* leftDewarper;
	FisheyeDewarper* rightDewarper;

	CameraModel* leftCamera;
	CameraModel* rightCamera; // replace with positions? 

private:
	// get the transaltion and rotation from cam2 to cam1
	void calcExtrinsics(cv::OutputArray mtx, cv::InputArray poseCam1, cv::InputArray poseCam2);

public:
	Stereopair(CameraModel*  lCam, FisheyeDewarper*  lDWarp, CameraModel* rCam, FisheyeDewarper* rDWarp);

	void fillMaps();
	// 
	int getRemapped(cv::Mat& left, cv::Mat& right, cv::Mat& leftRemapped, cv::Mat& rightRemapped);
	int getDisparity(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage);
	int getDepth(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage);
	void setStereoMethod(StereoMethod sm);
	// calculate and set the best roll,pitch,yaw values fir each camera in stereopir 
	void setOptimalDirecton();
	void setDirection();

};