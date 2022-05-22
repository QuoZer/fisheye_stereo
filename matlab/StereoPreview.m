close all;
global SHOW BasePath Scale RECALCULATE Models;

RECALCULATE = true;
SHOW = true;
BasePath = "D:\Work\Coding\Repos\fisheye_stereo\data\1080p[REAL]\1_Compar0.072m\"; 
Scale = 10; 
Models =["SCARA" "KB" "MEI" "DS"] % "REG" "ATAN"  "REAL_ATAN"
distances = [1 2 3 4 5 6 ]; % 0.5m: 4 5 6 7.5 9 10 11 12 12.5 13 14 15  //  0.3m: 1 2 4 5 6 7.5 10 // 0.05m: 1 2 3 4 5 6 8 10 12
% MEI0.1: 1 2 3 4 5 6 7 8
% ALL 1 2 3 4 5 6 7 8 9 

for ind = distances
      close all;
    stereo_preview(stereoParamsSCARA, ind, "SCARA")
    
end

function [ptCloud]  = stereo_preview(stereoParams, distance, type)
    global  BasePath;
    global SHOW;
    global Scale;
    %base_path = BasePath + string(distance) + "m\";  % compar0.3m
    
    targetDistance = distance * 0.25;
    
    imgLeft = BasePath + type + "/left/l_img_"+ type + string(distance-1) + ".png";
    imgRight = BasePath + type + "/right/r_img_" + type + string(distance-1) + ".png";

    lImage = imread(imgLeft);
    rImage = imread(imgRight);

    [frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, stereoParams);

    %figure;
    %imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
    %title('Rectified Video Frames');

    frameLeftGray  = rgb2gray(frameLeftRect);
    frameRightGray = rgb2gray(frameRightRect);
    
    disparityMapReg = disparitySGM(frameLeftGray, frameRightGray);          %disparityBM   disparitySGM
    figure;
    imshow(disparityMapReg, [0, 64]);
    title('Disparity Map'+string(targetDistance));
    colormap jet
    colorbar

    points3Dreg = reconstructScene(disparityMapReg, stereoParams);
    points3Dreg = points3Dreg ./ 1000;
    ptCloud = pointCloud(points3Dreg, 'Color', frameLeftRect);
%     if (SHOW) 
%         figure('Name',type+' full_scene')
%         pcshow(ptCloud);
%     end
    

    M = mean(ptCloud.Location(:,3))*Scale;
    if (SHOW)    
        disp(type+"MEAN: ")
        disp(M)
        % Visualize the point cloud
        figure('Name',type+' depth')
        pcshow(ptCloud);
        hold on
        hold off
        
    end
end