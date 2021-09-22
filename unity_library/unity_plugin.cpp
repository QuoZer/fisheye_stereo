// surroundView.cpp: определяет экспортированные функции для приложения DLL.

//#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include "surroundView.h"
#include <string>

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

    int initialize(int width, int height, int numOfImg)
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

        return 0;
    }

    int screenIndex = 0;

    int takeScreenshot(Color32** raw, int width, int height, int numOfCam, bool isShow)
    {
        cv::Mat sceen = cv::Mat(height, width, CV_8UC4, raw[numOfCam]);
        cvtColor(sceen, sceen, cv::COLOR_BGRA2RGB);
        flip(sceen, sceen, 0);
        screenIndex++;
        string path = "D:/Work/Coding/Repos/RTC_Practice/rtc_cvpractice/datasets/unity_chess/unity" + to_string(screenIndex) + "_shot.jpg";
        cv::imwrite(path, sceen);

        if (isShow)
        {
            imshow("OpenCV Screenshot", sceen);
        }

        return 0;
    }

    int getImages(Color32** raw, int width, int height, int numOfImg, bool isShow, FilterValues filter)
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
            frames[0] = colorFilter(frames[0], filter); // find spheres and print their coordinates

            UndistortFY(frames[1], frames[1]);          // undistort 
            frames[1] = colorFilter(frames[1], filter); // find spheres on undistorted
            /*
            cv::Mat hsvimg;
            cvtColor(frames[1], hsvimg, cv::COLOR_BGR2HSV);
            inRange(hsvimg, cv::Scalar(filter.HLow, filter.SLow, filter.VLow),
                cv::Scalar(filter.HHigh, filter.SHigh, filter.VHigh), frames[1]);
            */

            if (isShow)
            {
                for (int i = 0; i < numOfImg; i++)
                {
                    imshow(framesNames[i], frames[i]);
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

    cv::Mat colorFilter(const cv::Mat& src, FilterValues filter)
    {
        assert(src.type() == CV_8UC3);
        cv::Mat hsvimg, frame_threshold;

        cvtColor(src, hsvimg, cv::COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(hsvimg, cv::Scalar(filter.HLow, filter.SLow, filter.VLow), 
                        cv::Scalar(filter.HHigh, filter.SHigh, filter.VHigh), frame_threshold);
        
        float mapCoef = 6;
        //cv::Point2f centers 
        string ang_coordinates = findCoordinates(frame_threshold, src, true);
        string coordinates = "debug"; //" x: " + to_string(centers.x) + " y: " + to_string(centers.y);
        //string ang_coordinates = " yaw: " + to_string(centers.x / mapCoef) + " pitch: " + to_string(centers.y / mapCoef);
        string telemetry = " H: " + to_string(filter.HLow) + "/" + to_string(filter.HHigh) +  
                           " S: " + to_string(filter.SLow) + "/" + to_string(filter.SHigh) + 
                           " V: " + to_string(filter.VLow) + "/" + to_string(filter.VHigh);

        cv::putText(src, telemetry, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(255, 255, 255), 1, 8);

        cv::putText(src, coordinates, cv::Point(800, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
            cv::Scalar(255, 255, 255), 1, 8);
        cv::putText(src, ang_coordinates, cv::Point(800, 100), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(255, 255, 255), 1, 8);

        return src;
    }

    
}


string findCoordinates(const cv::Mat& binaryImage, const cv::Mat& imageToDrawOn, bool drawCenters)
{
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    string coordinates;
    string angles;

    cv::findContours(binaryImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    // get the moments
    vector<cv::Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i], false);
    }

    float x0 = 539.5;           // image center 
    float y0 = 539.5;
    float mapCoef = 1;      // a coef to map resolution into angle (linear)

    // get the centroid of figures.
    vector<cv::Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++)       // size is usually equals 1, left it here just in case
    {
        mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        angles = " x: " + to_string((mc[i].x - x0) / mapCoef) + " y: " + to_string(-(mc[i].y - y0)/ mapCoef); // '-'y due to inverted coordinates
    }

    // draw contours
    if (drawCenters)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(167, 151, 0); // B G R values
            drawContours(imageToDrawOn, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
            circle(imageToDrawOn, mc[i], 4, color, -1, 8, 0);
        }
    }
    //mc[0].x = mc[0].x - x0;     // applying offset AFTER drawing circles
    //mc[0].y = mc[0].y - y0;
    
    return angles;          // TODO: find out what's wrong with <Point>
}

void UndistortFY(cv::Mat& in, cv::Mat& undistorted)
{
    dewrapper(in, undistorted);
    return;                         // erase for 'honest' undistortion        // TODO: create a way to switch between undistortion models

    cv::Mat K = (cv::Mat_<double>(3, 3) << 286.47, 0, 516.37,
                                            0, 397.046, 544.712,
                                            0, 0, 1);      // MATLAB undistort values 
    cv::Mat D = (cv::Mat_<double>(1, 4) << -0.166, 2.129, -3.937, 2.365);
    cv::Size new_size = in.size() * 1;
    cv::Mat Knew = cv::Mat(cv::Matx33f(new_size.width / 4, 0, new_size.width / 2,
                                        0, new_size.height / 4, new_size.height / 2,
                                        0, 0, 1));

    cv::fisheye::undistortImage(in, undistorted, K, D, Knew, new_size);
}



// example from here: https://stackoverflow.com/questions/34316306/opencv-fisheye-calibration-cuts-too-much-of-the-resulting-image/53500300#53500300
#define PI 3.1415926536
cv::Point2f getInputPoint(int x, int y, int srcwidth, int srcheight)
{
    cv::Point2f pfish;
    float theta, phi, r, r2;
    cv::Point3f psph;
    float FOV = (float)PI / 270 * 180;
    float FOV2 = (float)PI / 270 * 180;     // ??? doesnt work the same way outside unity for some reason
    float width = srcwidth;
    float height = srcheight;

    // Polar angles
    theta = PI * (x / width - 0.5); // -pi/2 to pi/2
    phi = PI * (y / height - 0.5);  // -pi/2 to pi/2

    // Vector in 3D space
    psph.x = cos(phi) * sin(theta);
    psph.y = cos(phi) * cos(theta);
    psph.z = sin(phi) * cos(theta);

    // Calculate fisheye angle and radius
    theta = atan2(psph.z, psph.x);
    phi = atan2(sqrt(psph.x * psph.x + psph.z * psph.z), psph.y);

    r = width * phi / FOV;
    r2 = height * phi / FOV2;

    // Pixel in fisheye space
    pfish.x = 0.5 * width + r * cos(theta);
    pfish.y = 0.5 * height + r2 * sin(theta);
    return pfish;
}


void dewrapper(cv::Mat input, cv::Mat& outImagePtr)
{
    cv::Mat outImage(input.rows, input.cols, CV_8UC3);

    for (int i = 0; i < outImage.cols; i++)
    {
        for (int j = 0; j < outImage.rows; j++)
        {
            cv::Point2f inP = getInputPoint(i, j, input.cols, input.rows);      // find pixel on a distorted image corresponding to the desired one
            cv::Point inP2((int)inP.x, (int)inP.y);

            if (inP2.x >= input.cols || inP2.y >= input.rows)
                continue;

            if (inP2.x < 0 || inP2.y < 0)
                continue;
            cv::Vec3b color = input.at<cv::Vec3b>(inP2);
            outImage.at<cv::Vec3b>(cv::Point(i, j)) = color;
        }
    }

    outImagePtr = outImage;
}