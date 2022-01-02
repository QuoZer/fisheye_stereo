base_path = "D:\Work\Coding\Repos\RTC_Practice\fisheye_stereo\data\stereo_img\compar\plane7.3m0deg\";
target_roi = [-0.4 0.6 -0.6 0.5 0.78 0.82];
%  plane7.3m30deg
%%%     FISHEYE        %%%

imgRight = base_path + "fy_r_shot.jpg";
imgLeft = base_path + "fy_l_shot.jpg";

lImage = imread(imgLeft);
rImage = imread(imgRight);

[frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, stereoFisheyeParams);

%figure;
%imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
%title('Rectified Video Frames');

frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);
    
disparityMap = disparitySGM(frameLeftGray, frameRightGray);
% figure;
% imshow(disparityMap, [0, 64]);
% title('Disparity Map');
% colormap jet
% colorbar

points3D = reconstructScene(disparityMap, stereoFisheyeParams);
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);
indicies = findPointsInROI(ptCloud, target_roi);
ptCloud = select(ptCloud, indicies);

maxDistance = 0.02;
referenceVector = [0, 0, 1];
maxAngularDistance = 0;
[model1,inlierIndices,outlierIndices, meanError] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);
remainPtCloud = select(ptCloud,outlierIndices);
meanError   % mean square error in distance

% Visualize the point cloud
figure('Name','Fisheye depth')
pcshow(ptCloud);
hold on
plot(model1, 'color', 'white')
hold off

%%%     REGULAR        %%%

imgRight = base_path + "reg_r_shot.jpg";
imgLeft = base_path + "reg_l_shot.jpg";

lImage = imread(imgLeft);
rImage = imread(imgRight);

[frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, stereoRegularParams);

%figure;
%imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
%title('Rectified Video Frames');

frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);
    
disparityMapReg = disparitySGM(frameLeftGray, frameRightGray);
% figure;
% imshow(disparityMapReg, [0, 64]);
% title('Disparity Map');
% colormap jet
% colorbar

points3Dreg = reconstructScene(disparityMapReg, stereoRegularParams);
points3Dreg = points3Dreg ./ 1000;
ptCloud = pointCloud(points3Dreg, 'Color', frameLeftRect);
indicies = findPointsInROI(ptCloud, target_roi);
ptCloud = select(ptCloud, indicies);

maxDistance = 0.02;
referenceVector = [0, 0, 1];
maxAngularDistance = 0;
[model1,inlierIndices,outlierIndices, meanError] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);
remainPtCloud = select(ptCloud,outlierIndices);
meanError   % mean square error in distance

% Visualize the point cloud
figure('Name','Regular depth')
pcshow(ptCloud);
hold on
plot(model1, 'color', 'white')
hold off