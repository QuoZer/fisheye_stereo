close all;
global SHOW BasePath Scale Model StereoParams;

Model = "KB";
BasePath = "D:\Work\Coding\Repos\fisheye_stereo\data\1_4System0.1m\"; %+ Model + "\";  %0.1m\
Scale = 10; 
distances = [1 2 3 4 5 6 7 8 9 10];
StereoParams = stereoParams01m540p;

front = getPtCloud(0, "KB", 0);
right = getPtCloud(1, "KB", pi/2);
left = getPtCloud(2, "KB", -pi/2);
back = getPtCloud(3, "KB", pi);

% pcshow(front)
% hold on
% pcshow(right)
% pcshow(left)
% pcshow(back)
% hold off

mergeSize = 0.0015;
front_right = pcmerge(front, right, mergeSize);
front_right_left = pcmerge(front_right, left, mergeSize);
ptCloudScene = pcmerge(front_right_left, back, mergeSize);

% ROI
target_roi = [-2 2 -0.2 0.2 -2 2];
indicies = findPointsInROI(ptCloudScene, target_roi);
ptCloudBound = select(ptCloudScene, indicies);

pcwrite(ptCloudBound ,'object3d','PLYFormat','binary');
pcshow(ptCloudBound)

function ptCloudOut = getPtCloud(sp_index,type, z_rot_angle)
    global BasePath
    global StereoParams
    imgLeft = BasePath + type + "\" + string(sp_index) + "_l_img.png";
    imgRight = BasePath + type + "\" + string(sp_index) + "_r_img.png";

    lImage = imread(imgLeft);
    rImage = imread(imgRight);

    [frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, StereoParams);

    frameLeftGray  = rgb2gray(frameLeftRect);
    frameRightGray = rgb2gray(frameRightRect);

    disparityMapReg = disparitySGM(frameLeftGray, frameRightGray);          %disparityBM   disparitySGM

    points3Dreg = reconstructScene(disparityMapReg, StereoParams);
    points3Dreg = points3Dreg ./ 1000;
    ptc = pointCloud(points3Dreg, 'Color', frameLeftRect);
    
    % ptc transform
    theta = z_rot_angle;
    A = [cos(theta) 0 -sin(theta) 0; ...
        0 1 0 0; ...
         sin(theta) 0 cos(theta) 0; ...
         0 0 0 1];
    tform = affine3d(A);
    ptCloudOut = pctransform(ptc,tform);
end