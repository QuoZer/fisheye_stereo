#include "FisheyeDewarper.hpp"

FisheyeDewarper::FisheyeDewarper()
{

}

void FisheyeDewarper::createMaps()
{
    map1 = cv::Mat(oldSize, CV_32FC1, float(0));
    map2 = cv::Mat(oldSize, CV_32FC1, float(0));
}

void FisheyeDewarper::setSize(int oldWidth, int oldHeight, int newWidth, int  newHeight, float wideFov)
{
    oldSize.width = oldWidth;
    oldSize.height = oldHeight;
    newSize.width = newWidth;
    newSize.height = newHeight;
    createMaps();
    setFovWide(wideFov);
}

void FisheyeDewarper::setSize(cv::Size oldsize, cv::Size newsize, float wideFov)
{
    oldSize = oldsize;
    newSize = newsize;
    createMaps();
    setFovWide(wideFov);
}

void FisheyeDewarper::setFovWide(float wFov)
{
    xFov = wFov;
    yFov = wFov * newSize.height / newSize.width;           // * 9/16

   // std::cout << "FOV, x: " << xFov << " FOV, y: " << yFov << std::endl;
}

void FisheyeDewarper::setIntrinsics(double coeffs[4], cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor)
{
    polynom.assign(coeffs, coeffs+4);       // treats both values as pointers 

    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
    this->lambda = scaleFactor;
}

void FisheyeDewarper::setIntrinsics(double a1, double a2, double a3, double a4, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor)
{
    polynom.clear();
    polynom.push_back(a1);
    polynom.push_back(a2);
    polynom.push_back(a3);
    polynom.push_back(a4);

    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
    this->lambda = scaleFactor;
}

void FisheyeDewarper::setRpy(float yaw, float pitch, float roll)
{
    this->yaw   =    yaw * PI / 180;
    this->pitch = -pitch * PI / 180;
    this->roll  =   roll * PI / 180;

    // TODO: fillMaps() after angle update. Or should it be handled manually? 
}

std::vector<cv::Point> FisheyeDewarper::getBorder() {
    return this->frameBorder;
}

cv::Point2f FisheyeDewarper::projectWorldToFisheyeAtan(cv::Mat worldPoint)
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
    double xFocus = newSize.width / PI;
    double yFocus = newSize.height / PI;
    cv::Point projectionPoint(newSize.width / 2, newSize.height / 2);       //  initial value set to the image corner

    //  calculate the point location on fisheye image in central coordinates
    projectionPoint.x = xSign * xFocus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wy * wy) / (wx * wx) + 1);
    projectionPoint.y = ySign * yFocus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wx * wx) / (wy * wy) + 1);

    //  convert to corner coordinates
    projectionPoint.x = projectionPoint.x + newSize.width / 2;
    projectionPoint.y = -projectionPoint.y + newSize.height / 2;

    return projectionPoint;
}

cv::Point2f FisheyeDewarper::projectWorldToPinhole(cv::Mat cameraCoords)
{
    double xFovRad = xFov * PI / 180;
    double yFovRad = yFov * PI / 180;
    double xPinholeFocus = newSize.width  / (2 * tan(xFovRad / 2));
    double yPinholeFocus = newSize.height / (2 * tan(yFovRad / 2));

    float cx = cameraCoords.at<float>(0);
    float cy = cameraCoords.at<float>(1);
    float cz = cameraCoords.at<float>(2);
    
    cv::Point2f pinholePoint;
    pinholePoint.x = xPinholeFocus * cx / cz;
    pinholePoint.y = yPinholeFocus * cy / cz;

    return pinholePoint;
}

cv::Mat FisheyeDewarper::projectPinholeToWorld(cv::Point pixel)
{
    toCenter(pixel, newSize);

    float cz = 200.0;                                    // doesnt really affect much
    double xFovRad = xFov * PI / 180;                 
    double yFovRad = yFov * PI / 180;
    double xPinholeFocus = newSize.width / (2 * tan(xFovRad / 2));
    double yPinholeFocus = newSize.height / (2 * tan(yFovRad / 2));

    cv::Mat cameraCoords(1, 3, CV_32F, float(0));
    cameraCoords.at<float>(0) = pixel.x * cz / xPinholeFocus;
    cameraCoords.at<float>(1) = pixel.y * cz / yPinholeFocus;
    cameraCoords.at<float>(2) = cz;
    //std::cout << xPinholeFocus << " | " << yPinholeFocus << std::endl;
    return cameraCoords;
}

cv::Mat FisheyeDewarper::projectFisheyeToWorld(cv::Point pixel)
{
    cv::Vec2d undistPixel = stretchMatrix * ( cv::Vec2d(pixel.x, pixel.y) - centerOffset);      // TODO: delete
    //std::cout << stretchMatrix << " | " << cv::Vec2d(pixel.x, pixel.y) - centerOffset << " | " << lambda * undistPixel[0] << std::endl;
    cv::Mat cameraCoords(1, 3, CV_32F, float(0));
    double rho = norm(pixel);      // sqrt ( x^2 + y^2 )
    cameraCoords.at<float>(0) = lambda * undistPixel[0];
    cameraCoords.at<float>(1) = lambda * undistPixel[1];
    cameraCoords.at<float>(2) = lambda * (polynom[0] + polynom[1] * pow(rho, 2) + polynom[2] * pow(rho, 3) + polynom[3] * pow(rho, 4));

    return rotatePoint(cameraCoords);
}

cv::Point2d FisheyeDewarper::projectWorldToFisheye(cv::Mat worldPoint)
{
    double X = worldPoint.at<float>(0);
    double Y = worldPoint.at<float>(1);
    double Z = worldPoint.at<float>(2);
    double phi = atan2(Z, sqrt(X * X + Y * Y));
    double rho = 0;
    double error = 1; 

    int iter = 0;
    do 
    {
        double R = polynom[0];      // R = f(rho)
        for (int i = 1; i < polynom.size(); i++)            
        {
            R += polynom[i] * pow(rho, i+1);
            //std::cout << "R: " << R << std::endl;
        }

        error = atan2(R, rho) - phi;
        iter++;
        rho = rho + 200*error;
        //std::cout << "It " << iter << " Point: " << worldPoint << " | Rho: " << rho << " f(Rho): " << 424242 << " Error: " << error << std::endl;
    } while (std::abs(error) > 0.005 && iter < 100);
    errorsum += error; 

    lambda = sqrt(X * X + Y * Y) / rho;
    double u = X / lambda;      
    double v = Y / lambda; 

    cv::Point fypixel( stretchMatrix * cv::Vec2d(u, v) + centerOffset );        // technically could do toCorner's job, but I'll keep it simple for now
    toCorner(fypixel, oldSize);
    return fypixel;
}

cv::Mat FisheyeDewarper::rotatePoint(cv::Mat worldPoint)
{
    cv::Mat rotZ(cv::Matx33f(1, 0, 0,
                             0, cos(yaw), sin(yaw),
                             0, -sin(yaw), cos(yaw)));       
    cv::Mat rotX(cv::Matx33f(cos(pitch), 0, -sin(pitch),
                             0, 1, 0,                        
                             sin(pitch), 0, cos(pitch)));
    cv::Mat rotY(cv::Matx33f(cos(roll), -sin(roll), 0,      
                             sin(roll), cos(roll), 0,
                             0, 0, 1));
    return worldPoint * rotY * rotZ * rotX;         // calib3d/utils  proposes this order
}

// converting corner coordinates to the center ones
void FisheyeDewarper::toCenter(cv::Point& cornerPixel, cv::Size imagesize)
{
    cornerPixel.x = cornerPixel.x - imagesize.width / 2;
    cornerPixel.y = -cornerPixel.y + imagesize.height / 2;
}

// converting center coordinates to the corner ones
void FisheyeDewarper::toCorner(cv::Point& centerPixel, cv::Size imagesize)
{
    centerPixel.x =  centerPixel.x + imagesize.width / 2;
    centerPixel.y = -centerPixel.y + imagesize.height / 2;
}

void FisheyeDewarper::fillMaps(int mode)
{
    createMaps();
    frameBorder.clear();

    if (mode == SCARAMUZZA) fillMapsScaramuzza();
    if (mode == REV_SCARAMUZZA) fillMapsRevScaramuzza();
    if (mode == ATAN)       fillMapsAtan();

}

void FisheyeDewarper::fillMapsScaramuzza()
{
    for (int i = 0; i < newSize.width; i++)
    {
        for (int j = 0; j < newSize.height; j++)
        {
            cv::Mat worldPoint = projectPinholeToWorld(cv::Point(i, j));
            worldPoint = rotatePoint(worldPoint);
            cv::Point distPoint = projectWorldToFisheye(worldPoint);

            if (distPoint.x > oldSize.width - 1 || distPoint.x < 0 ||
                distPoint.y > oldSize.height - 1 || distPoint.y < 0)
            {
                continue;   // skips out of border points
            }

            // save distorted edge of the frame 
            if (((j == 0 || j == newSize.height - 1) && i % 100 == 0) ||
                ((i == 0 || i == newSize.width - 1) && j % 100 == 0))
            {
                frameBorder.push_back(cv::Point(distPoint.y, distPoint.x));
            }

            //map1.at<float>(distPoint.x, distPoint.y) =  j;
            //map2.at<float>(distPoint.x, distPoint.y) =  i;
            map1.at<float>(i, j) = distPoint.y;
            map2.at<float>(i, j) = distPoint.x;
        }
        if (i % 100 == 0) std::cout << "Collumn N" << i << std::endl;
    }
    std::cout << "Avg. error: " << errorsum / (newSize.area()) << std::endl;      
}

void FisheyeDewarper::fillMapsRevScaramuzza()
{
    for (int i = 0; i < oldSize.width; i++)
    {
        for (int j = 0; j < oldSize.height; j++)
        {
            cv::Point distPoint = reverseScaramuzza(cv::Point(i, j));

            if (distPoint.x > newSize.width - 1 || distPoint.x < 0 ||
                distPoint.y > newSize.height - 1 || distPoint.y < 0)
            {
                continue;   // skips out of border points
            }

            // save distorted edge of the frame 
            if (((j == 0 || j == oldSize.height - 1) && i % 100 == 0) ||
                ((i == 0 || i == oldSize.width - 1) && j % 100 == 0))
            {
                frameBorder.push_back(cv::Point(distPoint.y, distPoint.x));
            }

            map1.at<float>(i, j) = distPoint.y;
            map2.at<float>(i, j) = distPoint.x;
        }
        if (i%10==0) std::cout << "Collumn N" << i << std::endl;
    }
    std::cout << "Avg. error: " << errorsum / (1080 * 1080) << std::endl;       // HACK
}

void FisheyeDewarper::fillMapsAtan()
{
    for (int i = 0; i < newSize.width; i++)
    {
        for (int j = 0; j < newSize.height; j++)
        {
            cv::Point distPoint = projectWorldToFisheye(projectPinholeToWorld(cv::Point(i, j)));

            if (distPoint.x > oldSize.width - 1 || distPoint.x < 0 ||
                distPoint.y > oldSize.height - 1 || distPoint.y < 0)
            {
                continue;   // skips out of border points
            }

            // save distorted edge of the frame 
            if (((j == 0 || j == newSize.height - 1) && i % 100 == 0) ||
                ((i == 0 || i == newSize.width - 1) && j % 100 == 0))
            {
                frameBorder.push_back(cv::Point(distPoint.y, distPoint.x));
            }

            map1.at<float>(i, j) = distPoint.y;
            map2.at<float>(i, j) = distPoint.x;
        }
    }
}

cv::Mat FisheyeDewarper::dewrapImage(cv::Mat inputImage)
{
    cv::Mat remapped(newSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::remap(inputImage, remapped, map1, map2, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
    return remapped;
}

