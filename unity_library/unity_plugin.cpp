// surroundView.cpp: ���������� ���������������� ������� ��� ���������� DLL.

//#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include "surroundView.h"
#include <string>
#include "../cpp_project/FisheyeDewarper.cpp"

#define CAMERA_REGULAR 0
#define CAMERA_FISHEYE 1

using namespace std;

extern "C"
{
    //time's stuff 
    int startTime = 0;
    int stopTime = 0;
    int runTime = 0;

    //variables and objects for image transmition
    vector<Color32*> rawData; //raw data from virtual camera
    vector<cv::Mat> frames; // all frames
    vector<cv::cuda::GpuMat> framesGpu;
    vector<string> framesNames; //and their names
    cv::ocl::Context context;
    cv::ocl::Device device;

    bool gotFrames = false;
    bool isShowed = false;

    cv::Mat output;
    cv::Mat fisheyeDisparity;
    cv::Mat regularDisparity;

    FisheyeDewarper left_dewarper;      // dewarper library object
    FisheyeDewarper right_dewarper;
    // Creating an object of StereoSGBM algorithm
    //cv::Ptr<cv::StereoBM> stereo;
    cv::Ptr<cv::StereoSGBM> stereo;
    // initialize values for StereoSGBM parameters
    int numDisparities = 8;
    int blockSize = 5;
    int preFilterType = 1;
    int preFilterSize = 1;
    int preFilterCap = 31;
    int minDisparity = 0;
    int textureThreshold = 10;
    int uniquenessRatio = 15;
    int speckleRange = 0;
    int speckleWindowSize = 0;
    int disp12MaxDiff = -1;
    int dispType = CV_16S;



    int initialize(int width, int height, int numOfImg, int leftRot, int rightRot)
    {
        if (!cv::ocl::haveOpenCL())
        {
            return 2;
        }

        if (!context.create(cv::ocl::Device::TYPE_GPU))
        {
            return 3;
        }

        for (int i = 0; i < context.ndevices(); i++)
        {
            device = context.device(i);
        }

        // Select the first device
        cv::ocl::Device(context.device(0));

        if (numOfImg < 1)
        {
            return 1;
        }

        frames.clear();
        framesNames.clear();

        for (int i = 0; i < numOfImg; i++)
        {
            frames.push_back(cv::Mat(height, width, CV_8UC4));
            framesNames.push_back(to_string(i + 1) + "of" + to_string(numOfImg));
        }

        rawData.clear();

        for (int i = 0; i < numOfImg; i++)
        {
            rawData.push_back(0);
        }

        if (leftRot != 0 || rightRot != 0) {        // TODO: find a better way ASAP
            //  180 deg: 350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9)
            //  270 deg: 229.3778, -0.0016, 9.737 * pow(10, -7), -4.2154 * pow(10, -9)
            left_dewarper.setIntrinsics(350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9), cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1), 0.022);        // 270 deg coefs
            left_dewarper.setSize(cv::Size(1080, 1080), cv::Size(1080, 1080), 90);
            left_dewarper.setRpy(leftRot, 0, 0);
            left_dewarper.fillMaps(SCARAMUZZA);

            right_dewarper.setIntrinsics(350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9), cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1), 0.022);
            right_dewarper.setSize(cv::Size(1080, 1080), cv::Size(1080, 1080), 90);
            right_dewarper.setRpy(rightRot, 0, 0);
            right_dewarper.fillMaps(SCARAMUZZA);
        }
        stereo = cv::StereoSGBM::create();

        if (leftRot != 0 || rightRot != 0)
            cv::namedWindow("Fisheye disparity", cv::WindowFlags::WINDOW_AUTOSIZE);
        else
            cv::namedWindow("Regular disparity", cv::WindowFlags::WINDOW_AUTOSIZE);

        return 0;
    }

    int screenIndex = 0;

    int takeScreenshot(Color32** raw, int width, int height, int numOfCam, bool isShow)
    {
        cv::Mat sceen = cv::Mat(height, width, CV_8UC4, raw[numOfCam]);
        cvtColor(sceen, sceen, cv::COLOR_BGRA2RGB);
        flip(sceen, sceen, 0);
        screenIndex++;
        string path = "D:/Work/Coding/Repos/RTC_Practice/fisheye_stereo/data/stereo_img/" + to_string(screenIndex) + "_shot.jpg";
        cv::imwrite(path, sceen);

        if (isShow)
        {
            imshow("OpenCV Screenshot", sceen);
        }

        return 0;
    }

    int takeStereoScreenshot(Color32** raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow)
    {
        cv::Mat stereo = cv::Mat(height, 2 * width, CV_8UC4, cv::Scalar(0, 0, 0));
        cv::Mat cam1 = cv::Mat(height, width, CV_8UC4, raw[numOfCam1]);
        cv::Mat cam2 = cv::Mat(height, width, CV_8UC4, raw[numOfCam2]);

        cvtColor(cam1, cam1, cv::COLOR_BGRA2RGB);
        flip(cam1, cam1, 0);
        cvtColor(cam2, cam2, cv::COLOR_BGRA2RGB);
        flip(cam2, cam2, 0);

        //cv::hconcat(cam1, cam2, stereo);        // horizontal concatation
        //cam1.copyTo(stereo(cv::Rect(0, 0, width, height)));
        //cam2.copyTo(stereo(cv::Rect(width, 0, width, height)));
        cam1 = left_dewarper.dewrapImage(cam1);    // undistort 
        cam2 = right_dewarper.dewrapImage(cam2);

        screenIndex++;
        //      hardcoded image path((((
        string left_path = "D:/Work/Coding/Repos/RTC_Practice/fisheye_stereo/data/stereo_img/l" + to_string(screenIndex) + "_shot.jpg";
        string right_path = "D:/Work/Coding/Repos/RTC_Practice/fisheye_stereo/data/stereo_img/r" + to_string(screenIndex) + "_shot.jpg";

        cv::imwrite(left_path, cam1);
        cv::imwrite(right_path, cam2);

        if (isShow)
        {
            imshow("OpenCV Screenshot", cam1);
        }

        return 0;
    }

    int getImages(Color32** raw, int width, int height, int numOfImg, int imageType, bool isShow, SGBMparams sgbm)
    {
        if (numOfImg < 1)
        {
            gotFrames = false;
            return 1;
        }
        else
        {
#pragma omp parallel for
            for (int i = 0; i < numOfImg; i++)
            {
                frames[i] = cv::Mat(height, width, CV_8UC4, raw[i]);
                cvtColor(frames[i], frames[i], cv::COLOR_BGRA2RGB);
                flip(frames[i], frames[i], 0);

            }
            if (imageType == CAMERA_FISHEYE) {
                frames[0] = left_dewarper.dewrapImage(frames[0]);    // undistort 
                frames[1] = right_dewarper.dewrapImage(frames[1]);
            }
            fillStereoParams(sgbm);
            cv::Mat disparity = calculateDisparities(frames[0], frames[1], imageType);

            if (isShow)
            {
                for (int i = 0; i < numOfImg; i++)
                {
                    imshow(framesNames[i], frames[i]);
                    imshow("Disparity", disparity);
                    isShowed = true;
                }
            }
            else
            {
                if (isShowed)
                {
                    for (int i = 0; i < numOfImg; i++)
                    {
                        cv::destroyWindow(framesNames[i]);
                        isShowed = false;
                    }
                }
            }

            gotFrames = true;
        }

        return 0;
    }


    void processImage(unsigned char* data, int width, int height)
    {
        //Convert from RGB to ARGB
        cv::Mat argb_img;
        cvtColor(output, argb_img, cv::COLOR_RGB2BGRA);
        cv::flip(argb_img, argb_img, 0);
        vector<cv::Mat> bgra;
        split(argb_img, bgra);
        swap(bgra[0], bgra[3]);
        swap(bgra[1], bgra[2]);
        merge(bgra, argb_img);
        memcpy(data, argb_img.data, argb_img.total() * argb_img.elemSize());
    }

    void terminate()
    {
        rawData.clear();
        frames.clear();
        cv::destroyAllWindows();
    }
}

    void fillStereoParams(SGBMparams& sgbm)
    {
        stereo->setBlockSize(sgbm.blockSize * 2 + 5);
        stereo->setPreFilterCap(sgbm.preFilterCap);
        //stereo->setPreFilterSize(sgbm.preFilterSize*2+5);
        stereo->setP1(sgbm.preFilterSize);
        stereo->setMinDisparity(sgbm.minDisparity);
        stereo->setNumDisparities(sgbm.numDisparities * 16);
        //stereo->setTextureThreshold(sgbm.textureThreshold);
        stereo->setP2(sgbm.textureThreshold);
        stereo->setUniquenessRatio(sgbm.uniquenessRatio);
        stereo->setSpeckleWindowSize(sgbm.speckleWindowSize * 2);
        stereo->setSpeckleRange(sgbm.speckleRange);
        stereo->setDisp12MaxDiff(sgbm.disp12MaxDiff);
        // 2 pass expensive method
        // stereo->setMode(cv::StereoSGBM::MODE_HH);

    }

    cv::Mat calculateDisparities(cv::Mat leftImage, cv::Mat rightImage, int cameraType) {
        cv::Mat disp;
        // Converting images to grayscale
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2GRAY);
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2GRAY);

        // Calculating disparith using the StereoBM algorithm
        stereo->compute(leftImage, rightImage, disp);

        // Converting disparity values to CV_32F from CV_16S
        disp.convertTo(disp, CV_32F, 1.0);

        // Scaling down the disparity values and normalizing them 
        disp = (disp / 16.0f - (float)stereo->getMinDisparity()) / ((float)stereo->getNumDisparities());

        if (cameraType == 0) {
            fisheyeDisparity = disp;
            imshow("Regular disparity", fisheyeDisparity);
        }
        if (cameraType == 1) {
            regularDisparity = disp;
            imshow("Fisheye disparity", regularDisparity);
        }
        cv::waitKey(1);

        return disp;
    }
