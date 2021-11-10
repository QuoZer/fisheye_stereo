#pragma once
//#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

struct Color32
{
	uchar r;
	uchar g;
	uchar b;
	uchar a;
};

struct FilterValues
{
    uchar HLow;
    uchar HHigh;
    uchar SLow;
    uchar SHigh;
    uchar VLow;
    uchar VHigh;
};

struct SGBMparams
{
    uchar preFilterSize;
    uchar preFilterCap;
    uchar blockSize;
    uchar minDisparity;
    uchar numDisparities;
    uchar textureThreshold;
    uchar uniquenessRatio;
    uchar speckleRange;
    uchar disp12MaxDiff;
    uchar speckleWindowSize;
};

string findCoordinates(const cv::Mat& binaryImage, const cv::Mat& imageToDrawOn, bool drawCenters);
void UndistortFY(cv::Mat& in, cv::Mat& undistorted);
cv::Point2f getInputPoint(int x, int y, int srcwidth, int srcheight);
void fillStereoParams(SGBMparams& sgbm);
cv::Mat calculateDisparities(cv::Mat leftImage, cv::Mat rightImage);


extern "C"
{
	__declspec(dllexport) int initialize(int width, int height, int numOfImg, int leftRot, int rightRot);
    __declspec(dllexport) void terminate();
    __declspec(dllexport) int getImages(Color32** raw, int width, int height, int numOfImg, bool isShow, SGBMparams sgbm);
    __declspec(dllexport) void processImage(unsigned char* data, int width, int height);
    __declspec(dllexport) int takeScreenshot(Color32** raw, int width, int height, int numOfCam, bool isShow);
    __declspec(dllexport) int takeStereoScreenshot(Color32** raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow);

	cv::Mat colorFilter(const cv::Mat& src, FilterValues filter);
}

