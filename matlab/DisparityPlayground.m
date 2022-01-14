
base_path = "D:\Work\Coding\Repos\RTC_Practice\fisheye_stereo\data\stereo_img\compar\plane12.5m\";

targetDistance = 12.5;
target_roi = [-0.4 0.6 -0.6 0.46 targetDistance/10-0.05 targetDistance/10+0.05];

targetParamsVector = [0, 0, 1, -targetDistance/10];   % normal + distance
ref_model = planeModel(targetParamsVector);

SHOW = true;

%%%     FISHEYE        %%%

imgRight = base_path + "fy_r_shot.jpg";
imgLeft = base_path + "fy_l_shot.jpg";

lImage = imread(imgLeft);
rImage = imread(imgRight);
lImage = imcrop(lImage,[0 0 540 540]);
rImage = imcrop(rImage,[0 0 540 540]);


[frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, newFisheyeStereoParams);

figure;
imtool(stereoAnaglyph(frameLeftRect, frameRightRect));
title('Rectified Video Frames');

frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);
    
disparityMap = disparitySGM(frameLeftGray, frameRightGray);
% figure;
% imshow(disparityMap, [0, 64]);
% title('Disparity Map');
% colormap jet
% colorbar

points3D = reconstructScene(disparityMap, newFisheyeStereoParams);
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);
indicies = findPointsInROI(ptCloud, target_roi);
ptCloud = select(ptCloud, indicies);

% maxDistance = 0.02;
% referenceVector = [0, 0, 1];
% maxAngularDistance = 0;
% [model1,inlierIndices,outlierIndices, meanError] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
% plane1 = select(ptCloud,inlierIndices);
% remainPtCloud = select(ptCloud,outlierIndices);
% meanError   % mean square error in distance

disp("FISHEYE ERROR: ")
disp(findMSE(ptCloud,  ref_model))


if (SHOW)
    % Visualize the point cloud
    figure('Name','Fisheye depth')
    pcshow(ptCloud);
    hold on
    plot(ref_model, 'color', 'white')
    hold off  
end


%%%     REGULAR        %%%

imgRight = base_path + "reg_r_shot.jpg";
imgLeft = base_path + "reg_l_shot.jpg";

lImage = imread(imgLeft);
rImage = imread(imgRight);

[frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, newRegularStereoParams);

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

points3Dreg = reconstructScene(disparityMapReg, newRegularStereoParams);
points3Dreg = points3Dreg ./ 1000;
ptCloud = pointCloud(points3Dreg, 'Color', frameLeftRect);
indicies = findPointsInROI(ptCloud, target_roi);
ptCloud = select(ptCloud, indicies);

disp("REGULAR ERROR: ")
disp(findMSE(ptCloud,  ref_model))

if (SHOW)
    % Visualize the point cloud
    figure('Name','Regular depth')
    pcshow(ptCloud);
    hold on
    plot(ref_model, 'color', 'white')
    hold off
end


function [MSE] = findMSE(pt_cloud, plane_model)
    errorSum = 0.0;
%    figure;
%     Animated_Plot = animatedline;
    for elm = 1:pt_cloud.Count
        pnt = pt_cloud.Location(elm,:);
        pnt_error = findSquareError(pnt, plane_model);
        errorSum = errorSum + pnt_error;
        
        %hold on
        %addpoints(Animated_Plot, elm,  double(errorSum) );
        
%         if (mod(elm, 100) == 0)
%             double(pnt_error)
%             drawnow
%         end
        
    end
    
    MSE = sqrt(errorSum/pt_cloud.Count);
end

function [error] = findSquareError(point, plane)
    planePoint = [0, 0, -plane.Parameters(4)];  % plane center
    PQ = point - planePoint;
    error = (dot(PQ, plane.Normal) )^2;     % squared difference
end


% maxDistance = 0.02;
% referenceVector = [0, 0, 1];
% maxAngularDistance = 0;
% [model1,inlierIndices,outlierIndices, meanError] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
% plane1 = select(ptCloud,inlierIndices);
% remainPtCloud = select(ptCloud,outlierIndices);
% meanError   % print mean square error in distance