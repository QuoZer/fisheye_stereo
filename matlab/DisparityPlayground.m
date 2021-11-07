
imgRight = 'D:\Work\Coding\Repos\RTC_Practice\fisheye_stereo\data\stereo_img\Right_images\r18_shot.jpg';
imgLeft = 'D:\Work\Coding\Repos\RTC_Practice\fisheye_stereo\data\stereo_img\Left_images\l18_shot.jpg';

lImage = imread(imgLeft);
rImage = imread(imgRight);

[frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, stereoParams);

%figure;
%imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
%title('Rectified Video Frames');

frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);
    
disparityMap = disparitySGM(frameLeftGray, frameRightGray);
figure;
imshow(disparityMap, [0, 64]);
title('Disparity Map');
colormap jet
colorbar

points3D = reconstructScene(disparityMap, stereoParams);

Z = points3D(:,:,3);
%mask = repmat(Z > 950 & Z < 1100, [1,1,3]);
%frameLeftRect(~mask) = 0;
%imshow(frameLeftRect, 'InitialMagnification', 50)

% Convert to meters and create a pointCloud object
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);

% Create a streaming point cloud viewer
player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');

% Visualize the point cloud
view(player3D, ptCloud);