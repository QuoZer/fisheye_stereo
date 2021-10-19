#include "FisheyeDewrapper.hpp"

FisheyeDewrapper::FisheyeDewrapper()
{

}

void FisheyeDewrapper::createMaps()
{
    map1 = cv::Mat(oldSize, CV_32FC1, float(0));
    map2 = cv::Mat(oldSize, CV_32FC1, float(0));
}

void FisheyeDewrapper::setSize(int oldWidth, int oldHeight, int newWidth, int  newHeight, float wideFov)
{
    oldSize.width = oldWidth;
    oldSize.height = oldHeight;
    newSize.width = newWidth;
    newSize.height = newHeight;
    createMaps();
    setFovWide(wideFov);
}

void FisheyeDewrapper::setSize(cv::Size oldsize, cv::Size newsize, float wideFov)
{
    oldSize = oldsize;
    newSize = newsize;
    createMaps();
    setFovWide(wideFov);
}

void FisheyeDewrapper::setFovWide(float wFov)
{
    xFov = wFov;
    yFov = wFov * newSize.height / newSize.width;           // * 9/16

   // std::cout << "FOV, x: " << xFov << " FOV, y: " << yFov << std::endl;
}

void FisheyeDewrapper::setIntrinsics(double coeffs[4], cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor)
{
    polynom.assign(coeffs, coeffs+4);       // treats both values as pointers 

    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
    this->lambda = scaleFactor;
}

void FisheyeDewrapper::setIntrinsics(double a1, double a2, double a3, double a4, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor)
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

void FisheyeDewrapper::setRpy(float yaw, float pitch, float roll)
{
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;

    // TODO: fillMaps() after angle update. Or should it be handled manually? 
}

cv::Point2f FisheyeDewrapper::projectWorldToFisheye(cv::Mat worldPoint)
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

cv::Point2f FisheyeDewrapper::projectWorldToPinhole(cv::Mat cameraCoords)
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

cv::Mat FisheyeDewrapper::projectPinholeToWorld(cv::Point pixel)
{
    toCenter(pixel, oldSize);

    float cz = 2.0;                                    // doesnt really affect much
    double xFovRad = xFov * PI / 180;                 // assuming equal FOV on x & y
    double yFovRad = yFov * PI / 180;
    double xPinholeFocus = oldSize.width / (2 * tan(xFovRad / 2));
    double yPinholeFocus = oldSize.height / (2 * tan(yFovRad / 2));

    cv::Mat cameraCoords(1, 3, CV_32F, float(0));
    cameraCoords.at<float>(0) = pixel.x * cz / xPinholeFocus;
    cameraCoords.at<float>(1) = pixel.y * cz / yPinholeFocus;
    cameraCoords.at<float>(2) = cz;

    return cameraCoords;
}

cv::Mat FisheyeDewrapper::projectFisheyeToWorld(cv::Point pixel)
{
    cv::Vec2d undistPixel = stretchMatrix * ( cv::Vec2d(pixel.x, pixel.y) - centerOffset);      // TODO: invert stretchmatrix
    //std::cout << stretchMatrix << " | " << cv::Vec2d(pixel.x, pixel.y) - centerOffset << " | " << lambda * undistPixel[0] << std::endl;
    cv::Mat cameraCoords(1, 3, CV_32F, float(0));
    double rho = norm(pixel);      // sqrt ( x^2 + y^2 )
    cameraCoords.at<float>(0) = lambda * undistPixel[0];
    cameraCoords.at<float>(1) = lambda * undistPixel[1];
    cameraCoords.at<float>(2) = lambda * (polynom[0] + polynom[1] * pow(rho, 2) + polynom[2] * pow(rho, 3) + polynom[3] * pow(rho, 4));

    return rotatePoint(cameraCoords);
}

cv::Point FisheyeDewrapper::reverseSarcamuzza(cv::Point pixel)      // Fisheye -> Pinhole
{
    // converting corner coordinates to the center ones
    toCenter(pixel, newSize);

    cv::Point guessPoint(0, 0);      // image center
    double error = 100;
    double xSplit = newSize.width / 2;
    double ySplit = newSize.height / 2;
    
    do
    {   
        cv::Point2d guessProjection = projectWorldToPinhole(projectFisheyeToWorld(guessPoint));
        
        double xError = guessProjection.x - pixel.x;
        double yError = guessProjection.y - pixel.y;
        error = std::sqrt(xError*xError + yError*yError);
        //std::cout << guessPoint << " | " << guessProjection << std::endl;

        xSplit /= 1.5;
        ySplit /= 1.5;

        // picking the right quarter
        if (xError < 0) guessPoint.x = guessPoint.x + xSplit;
        else            guessPoint.x = guessPoint.x - xSplit;
        if (yError < 0) guessPoint.y = guessPoint.y + ySplit;
        else            guessPoint.y = guessPoint.y - ySplit;

        //std::cout << "Pixel: " << pixel << " Guess: " << guessProjection /* << " Error: " << error << " x| " << xError << " y| " << yError*/ << std::endl;

    } while (error > 1 && xSplit > 0.01);
    //std::cout << "Error: " << error << std::endl;
    errorsum += error;
    toCorner(guessPoint, newSize);
    
    return guessPoint;
}


/*
- Pinhole -> 3D coords
- Calc coef
- Fisheye -> 3D coords
*/
double center = false;

cv::Point2d FisheyeDewrapper::worldToFisheye(cv::Mat worldPoint)
{
    double X = worldPoint.at<float>(0);
    double Y = worldPoint.at<float>(1);
    double Z = worldPoint.at<float>(2);
    double phi = atan2(Z, sqrt(X * X + Y * Y));
    double R = phi;               // R = f(rho)
    double rho = 0;
    double error = 1; 

    double tan = Z / sqrt(X * X + Y * Y);

    int iter = 0;
    do 
    {
        //std::cout << "It " << iter << " | Rho: " << rho << " f(Rho): " << R << " Error: " << error << std::endl;
        R = polynom[0];
        for (int i = 1; i < polynom.size(); i++)            
        {
            R += polynom[i] * pow(rho, i+1);
            //std::cout << "R: " << R << std::endl;
        }

        error = atan2(R, rho) - phi;
        iter++;
        rho = rho + 200*error;
    } while (std::abs(error) > 0.005 && iter < 100);

    errorsum += error; 
    lambda = sqrt(X * X + Y * Y) / rho;
    double u = X / lambda;      // streth matrix + offset also go here
    double v = Y / lambda;  

    //u = u * 0.04 + v * 0;
    //v = u * 0 + v*0.04;
    //
    if (center) {
        std::cout << "Lambda: " << lambda << " U: " << u << " V: " << v << std::endl;
        center = false; 
    }
    if (X == 0 && Y == 0) center = true;
    
    cv::Point fypixel(u, v);
    toCorner(fypixel, newSize);
    return fypixel;
}

cv::Mat FisheyeDewrapper::rotatePoint(cv::Mat worldPoint)
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
    return worldPoint * rotY * rotX * rotZ;
}

// converting corner coordinates to the center ones
void FisheyeDewrapper::toCenter(cv::Point& cornerPixel, cv::Size imagesize)
{
    cornerPixel.x = cornerPixel.x - imagesize.width / 2;
    cornerPixel.y = -cornerPixel.y + imagesize.height / 2;
}

// converting center coordinates to the corner ones
void FisheyeDewrapper::toCorner(cv::Point& centerPixel, cv::Size imagesize)
{
    centerPixel.x =  centerPixel.x + imagesize.width / 2;
    centerPixel.y = -centerPixel.y + imagesize.height / 2;
}

void FisheyeDewrapper::fillMaps(int mode)
{
    createMaps();
    frameBorder.clear();

    if (mode == SARCAMUZZA) fillMapsSarcamuzza();
    if (mode == REV_SARCAMUZZA) fillMapsRevSarcamuzza();
    if (mode == ATAN)       fillMapsAtan();

}

void FisheyeDewrapper::fillMapsSarcamuzza()
{
    for (int i = 0; i < newSize.width; i++)
    {
        for (int j = 0; j < newSize.height; j++)
        {
            cv::Mat worldPoint = projectPinholeToWorld(cv::Point(i, j));
            worldPoint = rotatePoint(worldPoint);
            cv::Point distPoint = worldToFisheye(worldPoint);

            if (distPoint.x > oldSize.width - 1 || distPoint.x < 0 ||
                distPoint.y > oldSize.height - 1 || distPoint.y < 0)
            {
                continue;   // skips out of border points
            }

            // save distorted edge of the frame 
            if (((j == 0 || j == oldSize.height - 1) && i % 100 == 0) ||
                ((i == 0 || i == oldSize.width - 1) && j % 100 == 0))
            {
                frameBorder.push_back(cv::Point(distPoint.y, distPoint.x));
            }

            //map1.at<float>(distPoint.x, distPoint.y) =  j;
            //map2.at<float>(distPoint.x, distPoint.y) =  i;
            map1.at<float>(i, j) = distPoint.y;
            map2.at<float>(i, j) = distPoint.x;
        }
        if (i % 10 == 0) std::cout << "Collumn N" << i << std::endl;
    }
    std::cout << "Avg. error: " << errorsum / (1080 * 1080) << std::endl;       // HACK
}

void FisheyeDewrapper::fillMapsRevSarcamuzza()
{
    for (int i = 0; i < oldSize.width; i++)
    {
        for (int j = 0; j < oldSize.height; j++)
        {
            cv::Point distPoint = reverseSarcamuzza(cv::Point(i, j));

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

void FisheyeDewrapper::fillMapsAtan()
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

cv::Mat FisheyeDewrapper::dewrapImage(cv::Mat inputImage)
{
    cv::Mat remapped(newSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::remap(inputImage, remapped, map1, map2, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
    return remapped;
}
