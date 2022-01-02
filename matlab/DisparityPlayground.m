
imgRight = '..\data\stereo_img\Right_images\r2_shot.jpg';
imgLeft = '..\data\stereo_img\Left_images\l2_shot.jpg';

lImage = imread(imgLeft);
rImage = imread(imgRight);

[frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, stereoParams);

frameLeftGray  = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);
    
disparityMap = disparitySGM(frameLeftGray, frameRightGray);
%figure;
%imshow(disparityMap, [0, 64]);
%title('Disparity Map');
%colormap jet
%colorbar

points3D = reconstructScene(disparityMap, stereoParams);


Z = points3D(:,:,3);
%mask = repmat(Z > 950 & Z < 1100, [1,1,3]);
%frameLeftRect(~mask) = 0;
%imshow(frameLeftRect, 'InitialMagnification', 50)

% Convert to meters and create a pointCloud object
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);
% ptCloud = pcdenoise(ptCloud)

% roi = [-0.35 0.27 -0.17 0.22 0.50 0.56];
% indices = findPointsInROI(ptCloud,roi);
% ptCloudB = select(ptCloud,indices);
% figure
% pcshow(ptCloud.Location,[0.5 0.5 0.5])
% hold on
% pcshow(ptCloudB.Location,'r');
% legend('Point Cloud','Points within ROI','Location','southoutside','Color',[1 1 1])
% hold off


maxDistance = 0.02;
referenceVector = [0, 0, 1];
maxAngularDistance = 5;
[model1,inlierIndices,outlierIndices, meanError] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);
remainPtCloud = select(ptCloud,outlierIndices);
meanError   % mean square error in distance

figure
pcshow(plane1)
title('First Plane')
hold on
plot(model1, 'color', 'white')
hold off

% Create a streaming point cloud viewer
% player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
%    'VerticalAxisDir', 'down');


% Visualize normals
%normals = pcnormals(ptCloud);

%figure
%pcshow(ptCloud)
%title('Estimated Normals of Point Cloud')
%hold on
%x = ptCloud.Location(1:10:end,1:10:end,1);
%y = ptCloud.Location(1:10:end,1:10:end,2);
%z = ptCloud.Location(1:10:end,1:10:end,3);
%u = normals(1:10:end,1:10:end,1);
%v = normals(1:10:end,1:10:end,2);
%w = normals(1:10:end,1:10:end,3);
%quiver3(x,y,z,u,v,w, 0.03);
%hold off
% Visualize the point cloud
% view(player3D, ptCloud);