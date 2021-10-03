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
#include "FisheyeDewrapper.hpp"

const bool DETECT_CHESS = false;
const bool PUT_CIRCLES = false;
const bool FAST_METHOD = true;
const double PI = 3.1416;

int px_counter = 0;

using namespace cv;
using namespace std;

Point projectWorldToFisheye(Mat worldPoint, Size fisheyeSize);
Mat projectFisheyeToWorld(Point pixel);
Point2f projectWorldToPinhole(Mat cameraCoords, int width);

//  Trackbar parameters
int yawTrack = 90;
int yawTrack_max = 180;
int pitchTrack = 90;
int pitchTrack_max = 180;

//double pinholeFocus = 1080 / (2 * tan(90 / 2));  
// Global values to read trackbar values
Point globalPixel(0, 0);
Point globalReproj(0, 0);
//Mat globalCamCoords = Mat(1, 3, CV_32F, float(0));

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

static void on_trackbar(int,  void*)
{
    cout << "Yaw&pitch: " << yawTrack << " " << pitchTrack << endl;
    
}

// beta = brightness, alpha = contrast
void changeContrastAndBrightness(Mat& image, double alpha = 1, int beta = 0)
{
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            for (int c = 0; c < image.channels(); c++) {
                image.at<Vec3b>(y, x)[c] =
                    saturate_cast<uchar>(alpha * image.at<Vec3b>(y, x)[c] + beta);
            }
        }
    }
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
        size = 540;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 540;
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
    //waitKey();
    //waitKey(15);
    // End the number of arguments
    va_end(args);
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


// Defining callback functions for the trackbars to update parameter values
static void on_trackbar1(int, void*)
{
    stereo->setNumDisparities(numDisparities * 16);
    numDisparities = numDisparities * 16;
}

static void on_trackbar2(int, void*)
{
    stereo->setBlockSize(blockSize * 2 + 5);
    blockSize = blockSize * 2 + 5;
}

static void on_trackbar3(int, void*)
{
    stereo->setPreFilterType(preFilterType);
}

static void on_trackbar4(int, void*)
{
    stereo->setPreFilterSize(preFilterSize * 2 + 5);
    preFilterSize = preFilterSize * 2 + 5;
}

static void on_trackbar5(int, void*)
{
    stereo->setPreFilterCap(preFilterCap);
}

static void on_trackbar6(int, void*)
{
    stereo->setTextureThreshold(textureThreshold);
}

static void on_trackbar7(int, void*)
{
    stereo->setUniquenessRatio(uniquenessRatio);
}

static void on_trackbar8(int, void*)
{
    stereo->setSpeckleRange(speckleRange);
}

static void on_trackbar9(int, void*)
{
    stereo->setSpeckleWindowSize(speckleWindowSize * 2);
    speckleWindowSize = speckleWindowSize * 2;
}

static void on_trackbar10(int, void*)
{
    stereo->setDisp12MaxDiff(disp12MaxDiff);
}

static void on_trackbar11(int, void*)
{
    stereo->setMinDisparity(minDisparity);
}

void on_mouse(int e, int x, int y, int d, void* ptr)
{
    Point* p = (Point*)ptr;
    p->x = x;
    p->y = y;
}

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

// Creating an object of StereoSGBM algorithm
cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv,
        "{@input||input file - xml file with a list of the images, created with cpp-example-imagelist_creator tool}"
        "{help||show help}"
    );
    parser.about("This is a sample for fisheye camera undistortion. Example command line:\n"
        "    dewrapper imagelist.xml \n");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    const string inputFilename = parser.get<string>(0);

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

    int n_img = (int)image_list.size();
    cout << n_img << endl;

    // Creating a named window to be linked to the trackbars
    cv::namedWindow("disparity", cv::WINDOW_NORMAL);
    cv::namedWindow("disparityy", cv::WINDOW_NORMAL);
    cv::resizeWindow("disparityy", 600, 600);

    // Creating trackbars to dynamically update the StereoBM parameters
    cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackbar1);
    cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
    cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackbar3);
    cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackbar4);
    cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
    cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);
    cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
    cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
    cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
    cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);
    cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);

    Point mouse(0, 0);
    setMouseCallback("disparityy", on_mouse, &mouse);

    cv::Mat disp, disparity;

    bool depthSwitcher = false;
    int lastPitch = 0;
    int lastYaw = 0;
    int index = 2;

    Size origSize = Size(1080, 1080);       //imread(image_list[0], -1).size();
    Size newSize = origSize * 1;            // determines the size of the output image

    Mat map1(newSize, CV_32FC1, float(0));   // x map
    Mat map2(newSize, CV_32FC1, float(0));   // y map
    vector<Point> grid;                   // vectors of grid points
    vector<Point> gridDist;

    FisheyeDewrapper dewrapper;
    dewrapper.setCoefficents(350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9) );
    dewrapper.setSize(newSize);
    dewrapper.setFovWide(90);
    dewrapper.setRpy(0, 0, 0);


    while(true)         //  iterate through images       
    {
        Mat img = imread(image_list[index], -1);
        Mat right = img(Rect(0, 0, 1080, 1080)).clone();
        Mat left = img(Rect(1070, 0, 1080, 1080)).clone();

        if ((lastPitch != pitchTrack || lastYaw != yawTrack) && FAST_METHOD){
                                                       // destroy old points
            dewrapper.fillMaps(origSize);                      // fill new maps with current parameters. 
            cout << "Maps ready" << endl;

            lastPitch = pitchTrack;                                                       // remember parameters
            lastYaw = yawTrack;
        }

        Mat leftImageRemapped(newSize, CV_8UC3, Scalar(0, 0, 0));
        Mat rightImageRemapped(newSize, CV_8UC3, Scalar(0, 0, 0));
        leftImageRemapped = dewrapper.dewrapImage(left);
        rightImageRemapped = dewrapper.dewrapImage(right);
        
        bool textPut = false;
        // draw grid
        for each (Point center in gridDist)
        {
            if (!textPut) {
                Point textOrigin = center - Point(20,20);
                textPut = true;
            }
            circle(img, center, 4, Scalar(115, 25, 10), 3);
        }

        // Converting images to grayscale
        cv::cvtColor(leftImageRemapped, leftImageRemapped, cv::COLOR_BGR2GRAY);
        cv::cvtColor(rightImageRemapped, rightImageRemapped, cv::COLOR_BGR2GRAY);

        // Calculating disparith using the StereoBM algorithm
        stereo->compute(leftImageRemapped, rightImageRemapped, disp);

        // Converting disparity values to CV_32F from CV_16S
        disp.convertTo(disparity, CV_32F, 1.0);

        // Scaling down the disparity values and normalizing them 
        disparity = (disparity / 16.0f - (float)minDisparity) / ((float)numDisparities);
        // normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        string dispVal = to_string(disparity.at<float>(mouse));
        putText(disparity, dispVal, Point(30,950), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 255, 255));
        //ShowManyImages("Images", 2, leftImageRemapped, rightImageRemapped);
        // Displaying the disparity map
        cv::imshow("disparityy", disparity);

        if (depthSwitcher)
        {
            Mat image3D;
            double focus = 540.0;
            cv::Matx44d Q = cv::Matx44d(
                1.0, 0.0, 0.0, 0.0,
                0.0, -1.0, 0.0, 0.0,
                0.0, 0.0, focus * 0.05, 0.0,
                0.0, 0.0, 0.0, 0.1 );
            reprojectImageTo3D(disparity, image3D, Q);
            
            //cout << "WIP" << endl;
            cv::imshow("image 3d", image3D);

            depthSwitcher = false;
        }

        char key = (char)waitKey(1);
        switch (key){
        case 'r':
            index += 0;
            cout << "Reload" << endl;
            break;
        case 'q':
            index--;
            cout << "Prev image" << endl;
            break;
        case 'e':
            index++;
            cout << "Next image" << endl;
            break;
        case 'd':
            cout << "calculating depth" << endl;
            depthSwitcher = true;
            break;
        case 'z':
            exit(0);
        default: 
            index +=0;
            break;
        }
        
        if (index == n_img - 1 || index < 0) index = 0;
    }
}
