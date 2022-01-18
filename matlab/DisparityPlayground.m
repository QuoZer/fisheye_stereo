RECALCULATE = false;
global SHOW BasePath Scale;
SHOW = false;
BasePath = "D:\Work\Coding\Repos\RTC_Practice\fisheye_stereo\data\stereo_img\compar\plane";
Scale = 10; 
distances = [4 5 6 7.5 10 12.5 15]; % 1m: 4 5 6 7.5 10 12.5 15  //  0.3m: 1 2 4 5 6 7.5 10


if (RECALCULATE)
    regData = [];
    reg_disp = [];    
    fyData = [];
    fy_disp = [];
    
    for dst = distances
        %%%     FISHEYE        %%%
        [fy_e, fy_disp] = computePlaneError(newFisheyeStereoParams, dst, "fy", SHOW);
        fyData = [fyData [fy_e; fy_disp]];
        %%%     REGULAR        %%%
        [reg_e, reg_disp] = computePlaneError(newRegularStereoParams, dst, "reg", SHOW);
        regData = [regData [reg_e; reg_disp]];
        disp("Distance  ready")
    end
end
figure
errorbar(distances, regData(1,:), regData(2,:) ); hold on;
errorbar(distances, fyData(1,:), fyData(2,:) );
%createfigure(distances, [regErrors; fyErrors])



function [MSE, D]  = computePlaneError(stereoParams, distance, type)
    base_path = BasePath + string(distance) + "m\";  % compar0.3m

    targetDistance = distance;
    target_roi = [-0.4 0.6 -0.6 0.46 targetDistance/Scale-0.05 targetDistance/Scale+0.05];

    targetParamsVector = [0, 0, 1, -targetDistance/Scale];   % normal + distance
    ref_model = planeModel(targetParamsVector);

    imgRight = base_path + type + "_r_shot.jpg";
    imgLeft = base_path + type + "_l_shot.jpg";

    lImage = imread(imgLeft);
    rImage = imread(imgRight);

    [frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, stereoParams);

    %figure;
    %imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
    %title('Rectified Video Frames');

    frameLeftGray  = rgb2gray(frameLeftRect);
    frameRightGray = rgb2gray(frameRightRect);

    disparityMapReg = disparitySGM(frameLeftGray, frameRightGray);          %disparityBM
    
    % figure;
    % imshow(disparityMapReg, [0, 64]);
    % title('Disparity Map');
    % colormap jet
    % colorbar

    points3Dreg = reconstructScene(disparityMapReg, stereoParams);
    points3Dreg = points3Dreg ./ 1000;
    ptCloud = pointCloud(points3Dreg, 'Color', frameLeftRect);
    indicies = findPointsInROI(ptCloud, target_roi);
    ptCloud = select(ptCloud, indicies);
% 
    [MSE, D] = findMSE(ptCloud,  ref_model);

    if (SHOW)    
        disp(type+"ERROR: ")
        disp(MSE)
        % Visualize the point cloud
        figure('Name',type+' depth')
        pcshow(ptCloud);
        hold on
        plot(ref_model, 'color', 'white')
        hold off
    end
end

function [MSE, D] = findMSE(pt_cloud, plane_model)
    error_sum = 0.0;
    squaredError_sum = 0.0;
%    figure;
%     Animated_Plot = animatedline;
    for elm = 1:pt_cloud.Count
        pnt = pt_cloud.Location(elm,:);
        
        planePoint = [0, 0, -plane_model.Parameters(4)];  % plane center
        PQ = pnt - planePoint;
        error = dot(PQ, plane_model.Normal)*Scale;     % difference [meters]

        error_sum = error_sum + error;
        squaredError_sum = squaredError_sum + error^2;

        
    end
    
    MSE = sqrt(squaredError_sum/pt_cloud.Count);
    D = squaredError_sum/pt_cloud.Count;
    
end

function createfigure(X1, YMatrix1)
%CREATEFIGURE(X1, YMatrix1)
%  X1:  vector of x data
%  YMATRIX1:  matrix of y data

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple lines using matrix input to plot
plot1 = plot(X1,YMatrix1,'Marker','square');
set(plot1(1),'DisplayName','"Традиционная" стереопара');
set(plot1(2),'DisplayName','Предлагаемая стереосистема');

% Create ylabel
ylabel('Ошибка оценки  поверхности, м');

% Create xlabel
xlabel('Расстояние до поверхности, м');

% Uncomment the following line to preserve the X-limits of the axes
 xlim(axes1,[-0.000209902368405811 16.0517554642163]);
% Uncomment the following line to preserve the Y-limits of the axes
 ylim(axes1,[-3.8322756199846e-05 0.35351683918712]);
box(axes1,'on');
% Set the remaining axes properties
set(axes1,'XGrid','on','YGrid','on');
% Create legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.172415836709068 0.839227796053986 0.262158950099176 0.0602310215285902]);
end



% maxDistance = 0.02;
% referenceVector = [0, 0, 1];
% maxAngularDistance = 0;
% [model1,inlierIndices,outlierIndices, meanError] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
% plane1 = select(ptCloud,inlierIndices);
% remainPtCloud = select(ptCloud,outlierIndices);
% meanError   % print mean square error in distance