#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <time.h>
#include <cstdarg>

using namespace cv;
using namespace std;

int yawTrack = 90;
int yawTrack_max = 180;
int pitchTrack = 90;
int pitchTrack_max = 180;


static void cropUglyScreenshots(const vector<string>& list)
{
    int n_img = (int)list.size();
    for (int i = 0; i < n_img; ++i)
    {
        Mat img = imread(list[i], -1);
        Mat beauty = img(Rect(0, 0, 500, 500)).clone();
        string path = "t" + to_string(i) + "_crop.jpg";
        cout << path << endl;
        imwrite(path, beauty);
    }
}

static void on_trackbar(int, void*)
{
    cout << yawTrack <<  "  ";
    cout << pitchTrack << endl;
}

Point calculateFrameProjection(Vec3d worldPoint, Size fisheyeSize)
{
    Point projectionPont(fisheyeSize.width/2, fisheyeSize.height/2) ;       //  initial value set to the image angle
    if (worldPoint[0] == 0) worldPoint[0] += 0.0001;
    if (worldPoint[1] == 0) worldPoint[1] += 0.0001;
    if (worldPoint[2] == 0) worldPoint[2] += 0.0001;


    double focus = fisheyeSize.width / 3.14159;
    
    projectionPont.x = focus * atan(sqrt(worldPoint[0] * worldPoint[0] + worldPoint[1] * worldPoint[1]) / worldPoint[2])
        / sqrt( (worldPoint[1] * worldPoint[1]) / (worldPoint[0] * worldPoint[0]) + 1);
    projectionPont.y = focus * atan(sqrt(worldPoint[0] * worldPoint[0] + worldPoint[1] * worldPoint[1]) / worldPoint[2])
        / sqrt( (worldPoint[0] * worldPoint[0]) / (worldPoint[1] * worldPoint[1]) + 1);

    return projectionPont;
}

void ShowManyImages(string title, int nArgs, ...) {
    int size;
    int i;
    int m, n;
    int x, y;

    // w - Maximum number of images in a row
    // h - Maximum number of images in a column
    int w, h;

    // scale - How much we have to resize the image
    float scale;
    int max;

    // If the number of arguments is lesser than 0 or greater than 12
    // return without displaying
    if (nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if (nArgs > 14) {
        printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
        return;
    }
    // Determine the size of the image,
    // and the number of rows/cols
    // from number of arguments
    else if (nArgs == 1) {
        w = h = 1;
        size = 300;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 300;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 300;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }
    // Create a new 3 channel image
    Mat DispImage = Mat::zeros(Size(100 + size * w, 60 + size * h), CV_8UC3);
    // Used to get the arguments passed
    va_list args;
    va_start(args, nArgs);
    // Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
        // Get the Pointer to the IplImage
        Mat img = va_arg(args, Mat);

        // Check whether it is NULL or not
        // If it is NULL, release the image, and return
        if (img.empty()) {
            printf("Invalid arguments");
            return;
        }
        // Find the width and height of the image
        x = img.cols;
        y = img.rows;
        // Find whether height or width is greater in order to resize the image
        max = (x > y) ? x : y;

        // Find the scaling factor to resize the image
        scale = (float)((float)max / size);

        // Used to Align the images
        if (i % w == 0 && m != 20) {
            m = 20;
            n += 20 + size;
        }
        // Set the image ROI to display the current image
        // Resize the input image and copy the it to the Single Big Image
        Rect ROI(m, n, (int)(x / scale), (int)(y / scale));
        Mat temp; resize(img, temp, Size(ROI.width, ROI.height));
        temp.copyTo(DispImage(ROI));
    }

    // Create a new window, and show the Single Big Image
    //namedWindow(title, 1);
    imshow(title, DispImage);
    waitKey();
    // End the number of arguments
    va_end(args);
}

static void calcChessboardCorners(const Size& boardSize, const Size2d& squareSize, Mat& corners, string pattern)
{
    // corners has type of CV_64FC3
    corners.release();
    int n = boardSize.width * boardSize.height;
    corners.create(n, 1, CV_64FC3);
    Vec3d* ptr = corners.ptr<Vec3d>();
    for (int i = 0; i < boardSize.height; ++i)
    {
        for (int j = 0; j < boardSize.width; ++j)
        {
            if (pattern == "chessboaard")
                ptr[i*boardSize.width + j] = Vec3d(double(j * squareSize.width), double(i * squareSize.height), 0.0); //chessboard
            else
                ptr[i * boardSize.width + j] = Vec3d(double((2 * j + i % 2) * squareSize.width), double(i * squareSize.height), 0.0);   //circles
        }
    }
}

static bool detecChessboardCorners(const vector<string>& list, vector<string>& list_detected,
    vector<Mat>& imagePoints, Size boardSize, Size& imageSize)
{
    imagePoints.resize(0);
    list_detected.resize(0);
    int n_img = (int)list.size();
    Mat img;
    //namedWindow("Image View", 1);
    for (int i = 0; i < n_img; ++i)
    {
        cout << list[i] << "... ";
        Mat points;
        img = imread(list[i], IMREAD_GRAYSCALE);

       
        bool found = findChessboardCorners(img, boardSize, points);
        if (found)
        {
            cornerSubPix(img, points, Size(11, 11),         // 11 taken from calibration.cpp
                Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
            if (points.type() != CV_64FC2)
                points.convertTo(points, CV_64FC2);
            imagePoints.push_back(points);
            list_detected.push_back(list[i]);

            points.convertTo(points, CV_32F);       // the function checks for that
            drawChessboardCorners(img, boardSize, points, found);
        }
         //imshow("Image View", img);
         //char c = (char)waitKey();
         cout << (found ? "FOUND" : "NO") << endl;
    }
    if (!img.empty())
        imageSize = img.size();
    if (imagePoints.size() < 3)
        return false;
    else
        return true;
}


static bool detecCircles(const vector<string>& list, vector<string>& list_detected,
    vector<Mat>& imagePoints, Size boardSize, Size& imageSize)
{
    imagePoints.resize(0);
    list_detected.resize(0);
    int n_img = (int)list.size();
    Mat img;
    namedWindow("Image View", 1);
    for (int i = 0; i < n_img; ++i)
    {
        cout << list[i] << "... ";
        Mat points;
        img = imread(list[i], IMREAD_GRAYSCALE); //
        
        imshow("Image View", img);
        char c = (char)waitKey();
        // tweaking blob detector 
         SimpleBlobDetector::Params bParams;
        /* bParams.filterByArea = true;
         bParams.minArea = 2; bParams.maxArea = 10000;
         bParams.filterByInertia = true;
         bParams.minInertiaRatio = 0.01;*/
         Ptr<SimpleBlobDetector> bDetector = SimpleBlobDetector::create(bParams);
        //
        bool found = findCirclesGrid(img, boardSize, points, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING, bDetector);
        if (found)
        {
            if (points.type() != CV_64FC2)
                points.convertTo(points, CV_64FC2);
            imagePoints.push_back(points);
            list_detected.push_back(list[i]);
            //drawChessboardCorners(img, boardSize, points, found);
        }
        cout << (found ? "FOUND" : "NOO") << endl;
    }
    if (!img.empty())
        imageSize = img.size();
    if (imagePoints.size() < 3)
        return false;
    else
        return true;
}


static bool readStringList(const string& filename, vector<string>& l)
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
        l.push_back((string)*it);
    return true;
}

static void saveCameraParams(const string& filename, int flags, const Mat& cameraMatrix,
    const Mat& distCoeffs, const double xi, const vector<Vec3d>& rvecs, const vector<Vec3d>& tvecs,
    vector<string> detec_list, const Mat& idx, const double rms, const vector<Mat>& imagePoints)
{
    FileStorage fs(filename, FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm* t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty())
        fs << "nFrames" << (int)rvecs.size();

    if (flags != 0)
    {
        sprintf(buf, "flags: %s%s%s%s%s%s%s%s%s",
            flags & omnidir::CALIB_USE_GUESS ? "+use_intrinsic_guess" : "",
            flags & omnidir::CALIB_FIX_SKEW ? "+fix_skew" : "",
            flags & omnidir::CALIB_FIX_K1 ? "+fix_k1" : "",
            flags & omnidir::CALIB_FIX_K2 ? "+fix_k2" : "",
            flags & omnidir::CALIB_FIX_P1 ? "+fix_p1" : "",
            flags & omnidir::CALIB_FIX_P2 ? "+fix_p2" : "",
            flags & omnidir::CALIB_FIX_XI ? "+fix_xi" : "",
            flags & omnidir::CALIB_FIX_GAMMA ? "+fix_gamma" : "",
            flags & omnidir::CALIB_FIX_CENTER ? "+fix_center" : "");
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "xi" << xi;

    //cvWriteComment( *fs, "names of images that are acturally used in calibration", 0 );
    fs << "used_imgs" << "[";
    for (int i = 0; i < (int)idx.total(); ++i)
    {
        fs << detec_list[(int)idx.at<int>(i)];
    }
    fs << "]";

    if (!rvecs.empty() && !tvecs.empty())
    {
        Mat rvec_tvec((int)rvecs.size(), 6, CV_64F);
        for (int i = 0; i < (int)rvecs.size(); ++i)
        {
            Mat(rvecs[i]).reshape(1, 1).copyTo(rvec_tvec(Rect(0, i, 3, 1)));
            Mat(tvecs[i]).reshape(1, 1).copyTo(rvec_tvec(Rect(3, i, 3, 1)));
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << rvec_tvec;
    }

    fs << "rms" << rms;

    if (!imagePoints.empty())
    {
        Mat imageMat((int)imagePoints.size(), (int)imagePoints[0].total(), CV_64FC2);
        for (int i = 0; i < (int)imagePoints.size(); ++i)
        {
            Mat r = imageMat.row(i).reshape(2, imageMat.cols);
            Mat imagei(imagePoints[i]);
            imagei.copyTo(r);
        }
        fs << "image_points" << imageMat;
    }
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv,
        "{w||board width}"
        "{h||board height}"
        "{sw|1.0|square width}"
        "{sh|1.0|square height}"
        "{pt|chessboard|pattern: chessboard \ acircles}"
        "{o|out_camera_params.xml|output file}"
        "{fs|false|fix skew}"
        "{fp|false|fix principal point at the center}"
        "{@input||input file - xml file with a list of the images, created with cpp-example-imagelist_creator tool}"
        "{help||show help}"
    );
    parser.about("This is a sample for omnidirectional camera calibration. Example command line:\n"
        "    omni_calibration -w=6 -h=9 -sw=80 -sh=80 imagelist.xml \n");
    if (parser.has("help") || !parser.has("w") || !parser.has("h"))
    {
        parser.printMessage();
        return 0;
    }

    Size boardSize(parser.get<int>("w"), parser.get<int>("h"));
    Size2d squareSize(parser.get<double>("sw"), parser.get<double>("sh"));
    int flags = 0;
    if (parser.get<bool>("fs"))
        flags |= omnidir::CALIB_FIX_SKEW;
    if (parser.get<bool>("fp"))
        flags |= omnidir::CALIB_FIX_CENTER;
    const string outputFilename = parser.get<string>("o");
    const string inputFilename = parser.get<string>(0);
    const string pattern = parser.get<string>("pt");

    if (!parser.check())
    {
        parser.printErrors();
        return -1;
    }

    // get image name list
    vector<string> image_list, detec_list;
    if (!readStringList(inputFilename, image_list))
    {
        cout << "Can not read imagelist" << endl;
        return -1;
    }

    //  crops images in a list and saves them 
    //cropUglyScreenshots(image_list);
    //

    // find corners in images
    // some images may be failed in automatic corner detection, passed cases are in detec_list
    cout << "Detecting patterns (" << image_list.size() << ")" << endl;
    vector<Mat> imagePoints;
    Size imageSize;
    if (pattern == "chessboard")
    {
        if (!detecChessboardCorners(image_list, detec_list, imagePoints, boardSize, imageSize))
        {
            cout << "Not enough corner detected images" << endl;
            return -1;
        }
    }
    else if (pattern == "acircles")
    {
        if (!detecCircles(image_list, detec_list, imagePoints, boardSize, imageSize))    //circles grid modification
        {
            cout << "Not enough corner detected images" << endl;
            return -1;
        }
    }
    else cout << "Wrong pattern name" << endl;


    // calculate object coordinates
    vector<Mat> objectPoints;
    Mat object;
    calcChessboardCorners(boardSize, squareSize, object, pattern);
    for (int i = 0; i < (int)detec_list.size(); ++i)
        objectPoints.push_back(object);

    // run calibration, some images are discarded in calibration process because they are failed
    // in initialization. Retained image indexes are in idx variable.
    Mat K, D, xi, idx;
    
    vector<Vec3d> rvecs, tvecs;
    double _xi, rms;
    TermCriteria criteria(3, 200, 1e-8);
    rms = omnidir::calibrate(objectPoints, imagePoints, imageSize, K, xi, D, rvecs, tvecs, flags, criteria, idx);
    _xi = xi.at<double>(0);
    cout << "Saving camera params to " << outputFilename << endl;
    saveCameraParams(outputFilename, flags, K, D, _xi,
        rvecs, tvecs, detec_list, idx, rms, imagePoints);

    int n_img = (int)image_list.size();
    cout << n_img << endl;

    Size new_size = imageSize ;
    
    Mat Knew = cv::Mat(cv::Matx33f(imageSize.width / 4, 0, imageSize.width / 2,
                                    0, imageSize.height / 4, imageSize.height / 2,
                                    0, 0, 1));
    

    namedWindow("Images", 1);
    createTrackbar("Yaw", "Images", &yawTrack, yawTrack_max, on_trackbar);
    createTrackbar("Pitch", "Images", &pitchTrack, pitchTrack_max, on_trackbar);

    on_trackbar(yawTrack, 0);

    for (int i = 0; i < n_img; ++i)
    {
        Mat imageUndistorted;
        Mat img = imread(image_list[i], -1);
        
        Mat R = cv::Mat::eye(3, 3, CV_32FC1);
        Mat Mapx, Mapy;
        Mat P(3, 3, CV_32FC1);
        P = K;
        Mat Kn = K;

        Size new_size = imageSize * 2;

        Mat Knew = cv::Mat(cv::Matx33f(imageSize.width / (yawTrack*2), 0, imageSize.width / yawTrack,
            0, imageSize.height / (yawTrack*2), imageSize.height / yawTrack,
            0, 0, 1));
        //cout << Knew;

        double yaw = (yawTrack - 90) * 3.1416 / 180;  //(15 - yawTrack * 15) * 3.1416 / 180;
        double pitch = -(pitchTrack - 90) * 3.1416 / 180;
        cv::Mat vecMat = (Mat_<float>(1,3) << 0, 0, 8);
        cv::Mat rotY(cv::Matx33f(cos(yaw), sin(yaw), 0,
                                -sin(yaw), cos(yaw), 0,
                                               0, 0, 1));   // weird up-facing Y

        cv::Mat rotX(cv::Matx33f(cos(yaw), 0, -sin(yaw),
                                 0,          1,      0,
                                 sin(yaw), 0, cos(yaw)   ));   // horizontal x

        cv::Mat rotZ(cv::Matx33f(1, 0, 0,
                                 0, cos(pitch), sin(pitch),
                                 0, -sin(pitch), cos(pitch)));   // 
        
        cout << vecMat * rotZ * rotX;  // * rotY
        cout << calculateFrameProjection(vecMat, imageSize) << endl;

        //omnidir::initUndistortRectifyMap(Kn, D, xi, R, P, new_size, CV_32FC1, Mapx, Mapy, cv::omnidir::RECTIFY_CYLINDRICAL);// , Knew, new_size); RECTIFY_PERSPECTIVE
        //remap(img, imageUndistorted, Mapx, Mapy, INTER_CUBIC);
        //undistort(img, imageUndistorted, Kn, D);
        omnidir::undistortImage(img, imageUndistorted, K, D, xi, omnidir::RECTIFY_CYLINDRICAL ); //RECTIFY_PERSPECTIVE  ,Knew, new_size, R 
        ShowManyImages("Images", 2, img, imageUndistorted);
        //char c = (char)waitKey();

        if (i == n_img - 1) i = 0;
    }
}
