#pragma once 
#include "CameraModel.h"


class PinholeModel : public CameraModel
{
	/* Parameters are as in parent */

public:	///* Projection functions *///
	PinholeModel();
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);
	void setIntrinsics(cv::Size newsize, float wideFov);
};


class ScaramuzzaModel : public CameraModel
{
private:	///* Intrinsics *///
	std::vector <double> scara_polynom;		// Scaramuzza model coefficients
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 

public:	///* Projection functions *///
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);

public:
	void setIntrinsics(std::initializer_list<double>, cv::Vec2d centerOffset, 
		cv::Matx22d stretchMatrix, double scaleFactor);
};

class AtanModel : public CameraModel
{
private:	///* Intrinsics *///
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 

private:	///* Projection functions *///
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	//cv::Mat projectPixelToWorld(cv::Point pixel) {};

public:
	//void setIntrinsics(cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor);

};

class MeiModel : public CameraModel
{

};