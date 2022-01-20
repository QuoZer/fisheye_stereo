close all;
global SHOW BasePath Scale;

RECALCULATE = true;
SHOW = false;
BasePath = "D:\Work\Coding\Repos\RTC_Practice\fisheye_stereo\data\stereo_img\compar\plane";  %0.05m
Scale = 10; 
distances = [ 5 6 7.5 9 10 11 12 12.5 13 14 15 ]; % 0.5m: 4 5 6 7.5 9 10 11 12 12.5 13 14 15  //  0.3m: 1 2 4 5 6 7.5 10 // 0.05m: 1 2 3 4 5 6 8 10 12


if (RECALCULATE)
    regData = [];
    reg_disp = [];    
    fyData = [];
    fy_disp = [];
    means = [];
    
    for dst = distances
        %%%     FISHEYE        %%%
        [fy_e, fy_disp, fy_mean] = computePlaneError(newFisheyeStereoParams, dst, "fy");
        fyData = [fyData [fy_e; fy_disp; fy_mean]];
        %%%     REGULAR        %%%
        [reg_e, reg_disp, reg_mean] = computePlaneError(newRegularStereoParams, dst, "reg");
        regData = [regData [reg_e; reg_disp; reg_mean]];
        disp("Distance  ready")
    end
end

createfigure(distances, [regData; fyData])


function [MSE, D, M]  = computePlaneError(stereoParams, distance, type)
    global  BasePath;
    global SHOW;
    global Scale;
    base_path = BasePath + string(distance) + "m\";  % compar0.3m

    targetDistance = distance;
    target_roi = [-0.4 0.6 -0.6 0.46 targetDistance/Scale-0.1 targetDistance/Scale+0.1];

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

    disparityMapReg = disparityBM(frameLeftGray, frameRightGray);          %disparityBM   disparitySGM
   

    points3Dreg = reconstructScene(disparityMapReg, stereoParams);
    points3Dreg = points3Dreg ./ 1000;
    ptCloud = pointCloud(points3Dreg, 'Color', frameLeftRect);
    if (SHOW) 
        figure('Name',type+' full_scene'+string(targetDistance))
        pcshow(ptCloud);
    end
    indicies = findPointsInROI(ptCloud, target_roi);
    ptCloud = select(ptCloud, indicies);

    [MSE, D] = findMSE(ptCloud,  ref_model);
%     MSE = 0;
%     D  = 0;
    M = mean(ptCloud.Location(:,3));
    
    if (SHOW)    
        disp(type+"ERROR: ")
        disp(MSE)
        % Visualize the point cloud
        figure('Name',type+' depth'+string(targetDistance))
        pcshow(ptCloud);
        hold on
        plot(ref_model, 'color', 'white')
        hold off
        
        figure;
        imshow(disparityMapReg, [0, 64]);
        title('Disparity Map'+string(targetDistance));
        colormap jet
        colorbar
    end
end

function [MSE, D] = findMSE(pt_cloud, plane_model)
    global Scale;
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
YMatrix1;
plot1  = errorbar(X1, YMatrix1(1,:), YMatrix1(2,:),'Marker','diamond',...
    'LineStyle','--',...
    'LineWidth',2 ); hold on;
plot2 = errorbar(X1, YMatrix1(4,:), YMatrix1(5,:),'Marker','diamond',...
    'LineStyle','--',...
    'LineWidth',2 ); 


xFit = linspace(0,50, 50);
coefficients = polyfit(X1, YMatrix1(1,:), 1);
yFit = polyval(coefficients , xFit);
plot_l1 = plot(xFit, yFit, 'b-', 'LineWidth', 1); % Plot fitted line.

coefficients = polyfit(X1, YMatrix1(4,:), 1);
yFit = polyval(coefficients , xFit);
plot_l2 = plot(xFit, yFit, 'r-', 'LineWidth', 1); % Plot fitted line.

set(plot_l1,'Color',[0 0 1]);
set(plot_l2,...
    'Color',[0.850980392156863 0.325490196078431 0.098039215686274]);

%plot1 = plot(X1,YMatrix1,'Marker','square');
set(plot1(1),'DisplayName','"Традиционная" стереопара');
set(plot2(1),'DisplayName','Предлагаемая стереосистема');

% Create ylabel
ylabel('Среднеквадратичная ошибка оценки  поверхности, м');

% Create xlabel
xlabel('Расстояние до поверхности, м');

% Uncomment the following line to preserve the X-limits of the axes
 xlim(axes1,[-0.000209902368405811 16.0517554642163]);
% Uncomment the following line to preserve the Y-limits of the axes
 ylim(axes1,[-3.8322756199846e-05 0.55351683918712]);
box(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',14,'XGrid','on','YGrid','on');
% Create legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.172415836709068 0.839227796053986 0.262158950099176 0.0602310215285902]);
end
