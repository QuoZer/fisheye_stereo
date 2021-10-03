#include "FisheyeDewrapper.hpp"

FisheyeDewrapper::FisheyeDewrapper()
{

}

void FisheyeDewrapper::createMaps()
{
    map1 = cv::Mat(newSize, CV_32FC1, float(0));
    map2 = cv::Mat(newSize, CV_32FC1, float(0));
}

void FisheyeDewrapper::setSize(int width, int height)
{
    newSize.width = width;
    newSize.height = height;
    createMaps();
}

void FisheyeDewrapper::setSize(cv::Size size)
{
    newSize = size;
    createMaps();
}

void FisheyeDewrapper::setFov(float x, float y)
{
    xFov = x;
    yFov = y;

}

void FisheyeDewrapper::setFovWide(float wFov)
{
    xFov = wFov;
    yFov = wFov * 9 / 16;
}

void FisheyeDewrapper::setCoefficents(double coeffs[4])
{
    polynom[0] = coeffs[0];
    polynom[1] = coeffs[1];
    polynom[2] = coeffs[2];
    polynom[3] = coeffs[3];
}

void FisheyeDewrapper::setCoefficents(double a1, double a2, double a3, double a4)
{
    polynom[0] = a1;
    polynom[1] = a2;
    polynom[2] = a3;
    polynom[3] = a4;
}

void FisheyeDewrapper::setRpy(float yaw, float pitch, float roll)
{
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;

    // TODO: fillMaps() after angle update
}

cv::Point2f FisheyeDewrapper::projectWorldToFisheye(cv::Mat worldPoint, cv::Size fisheyeSize)
{
    float wx = worldPoint.at<float>(0);
    float wy = worldPoint.at<float>(1);
    float wz = worldPoint.at<float>(2);
    //  sqrt destroys signs so I remember them here
    int8_t xSign = 1, ySign = 1;
    if (wx == 0) wx += 0.0001;
    else if (wx < 0) xSign = -1;
    if (wy == 0) wy += 0.0001;
    else if (wy < 0) ySign = -1;
    if (wz == 0) wz += 0.0001;
    //  fisheye focus
    double focus = fisheyeSize.width / PI;
    cv::Point projectionPoint(fisheyeSize.width / 2, fisheyeSize.height / 2);       //  initial value set to the image angle
    //  calculate the point location on fisheye image in central coordinates
    projectionPoint.x = xSign * focus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wy * wy) / (wx * wx) + 1);
    projectionPoint.y = ySign * focus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wx * wx) / (wy * wy) + 1);
    //  convert to angle coordinates
    projectionPoint.x = projectionPoint.x + fisheyeSize.width / 2;
    projectionPoint.y = -projectionPoint.y + fisheyeSize.height / 2;

    return projectionPoint;
}

cv::Point2f FisheyeDewrapper::projectWorldToPinhole(cv::Mat cameraCoords, cv::Size imgSize)
{
    double fovRad = xFov * PI / 180;
    double pinholeFocus = imgSize.width / (2 * tan(fovRad / 2));

    float cx = cameraCoords.at<float>(0);
    float cy = cameraCoords.at<float>(1);
    float cz = cameraCoords.at<float>(2);
    
    cv::Point2f pinholePoint;
    pinholePoint.x = pinholeFocus * cx / cz;
    pinholePoint.y = pinholeFocus * cy / cz;

    return pinholePoint;
}

cv::Mat FisheyeDewrapper::projectPinholeToWorld(cv::Point pixel, cv::Size imgSize)
{
    pixel.x =   pixel.x - imgSize.width / 2;         // converting angle coordinates to the center ones
    pixel.y = - pixel.y + imgSize.height / 2;

    float cz = 2;                                   // doesnt really affect much
    double fovRad = xFov * PI / 180;                 // assuming equal FOV on x & y
    double pinholeFocus = imgSize.width / (2 * tan(fovRad / 2));

    cv::Mat cameraCoords(1, 3, CV_32F, float(0));
    cameraCoords.at<float>(0) = pixel.x * cz / pinholeFocus;
    cameraCoords.at<float>(1) = pixel.y * cz / pinholeFocus;
    cameraCoords.at<float>(2) = cz;

    return rotatePoints(cameraCoords);
}

cv::Mat FisheyeDewrapper::projectFisheyeToWorld(cv::Point pixel)
{
    cv::Mat direction(1, 3, CV_32F, float(0));
    //double a[] = { 350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9) };       // Sarcamuzza coeffs from MATLAB
    double scale = 0.0222;         // lambda scale factor
    double rho = norm(pixel);      // sqrt ( x^2 + y^2 )

    direction.at<float>(0) = scale * pixel.x;
    direction.at<float>(1) = scale * pixel.y;
    direction.at<float>(2) = scale * (polynom[0] + polynom[1] * pow(rho, 2) + polynom[2] * pow(rho, 3) + polynom[3] * pow(rho, 4));

    return direction;
}

cv::Point FisheyeDewrapper::reverseSarcamuzza(cv::Point pixel)
{
    cv::Point guessPoint(0, 0);
    double error = 100;
    int coef;

    do
    {   
        cv::Point2f guessProjection = projectWorldToPinhole(projectFisheyeToWorld(guessPoint), newSize);

        double xError = guessProjection.x - pixel.x;
        double yError = guessProjection.y - pixel.y;
        error = std::sqrt(xError*xError + yError*yError);
            
        /* TODO complete the loop  */

        double phi = std::atan2(yError, xError);
        guessPoint.x += xError 

    } while (error > 0.1);
    
    return cv::Point();
}

cv::Mat FisheyeDewrapper::rotatePoints(cv::Mat worldPoints)
{
    cv::Mat rotZ(cv::Matx33f(1, 0, 0,
        0, cos(yaw), sin(yaw),
        0, -sin(yaw), cos(yaw)));       // roll?
    cv::Mat rotX(cv::Matx33f(cos(pitch), 0, -sin(pitch),
        0, 1, 0,                        // pitch
        sin(pitch), 0, cos(pitch)));
    cv::Mat rotY(cv::Matx33f(cos(roll), -sin(roll), 0,      // yaw?
        sin(roll), cos(roll), 0,
        0, 0, 1));
    return worldPoints * rotY * rotX * rotZ;
}

void FisheyeDewrapper::fillMaps(cv::Size origSize)
{
    frameBorder.clear();
    for (int i = 0; i < newSize.width; i++)
    {
        for (int j = 0; j < newSize.height; j++)
        {
            cv::Point distPoint = projectWorldToFisheye(projectPinholeToWorld(
                cv::Point(i, j), origSize),
                newSize);

            if (distPoint.x > origSize.width - 1 || distPoint.x < 0 ||
                distPoint.y > origSize.height - 1 || distPoint.y < 0)
            {
                continue;   // skips out of border points
            }

            // save distorted edge of the frame 
            if (((j == 0 || j == newSize.height - 1) && i % 100 == 0) ||
                ((i == 0 || i == newSize.width - 1) && j % 100 == 0)  )
            {
                frameBorder.push_back(cv::Point(distPoint.y, distPoint.x));
            }

            map1.at<float>(i, j) = distPoint.y;
            map2.at<float>(i, j) = distPoint.x;
        }
    }
}

cv::Mat FisheyeDewrapper::dewrapImage(cv::Mat inputImage)
{
    cv::Mat remapped(newSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::remap(inputImage, remapped, map1, map2, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
    return remapped;
}
