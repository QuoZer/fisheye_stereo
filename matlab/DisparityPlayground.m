close all;
global SHOW BasePath Scale Model Models;

RECALCULATE = true;
SHOW = false;
BasePath = "D:\Work\Coding\Repos\fisheye_stereo\data\1_Compar0.1m\"; %+ Model + "\";  %0.1m\
Scale = 10; 
Models =["REG" "ATAN" "MEI" "SCARA" "KB" "REAL_ATAN"]
distances = [1 2 3 4 5 6 7 8 9 10]; % 0.5m: 4 5 6 7.5 9 10 11 12 12.5 13 14 15  //  0.3m: 1 2 4 5 6 7.5 10 // 0.05m: 1 2 3 4 5 6 8 10 12
% MEI0.1: 1 2 3 4 5 6 7 8
% ALL 1 2 3 4 5 6 7 8 9 10

% diffImageAll(Models, 5)
computeAllModels(Models, stereoParams01m540p, distances, RECALCULATE);


%%%%%% FUNCTIONS %%%%%%


function diffImageAll(modelDict, distance )
    global BasePath
    sample_path = BasePath + "REG/l_img_REG" + string(distance-1) + ".png";
    sample_img = imread(sample_path);
    sample_img = rgb2gray(sample_img);
    for type = modelDict
        imgLeft = BasePath + type + "/r_img_" + type + string(distance-1) + ".png";
        lImage = imread(imgLeft);
        
        lImage = rgb2gray(lImage);
        diffImage = lImage - sample_img;
        
%         pout_imadjust = imadjust(diffImage);
%         pout_histeq = histeq(diffImage);
        pout_adapthisteq = adapthisteq(diffImage);
        
%         montage({diffImage,pout_imadjust,pout_histeq,pout_adapthisteq},"Size",[1 4])
%         title("Original Image and Enhanced Images using imadjust, histeq, and adapthisteq")
        
        figure('Name', 'Diff Image '+type);
        imshow(pout_adapthisteq)
    end
end

function computeAllModels(modelDict, stereoParams, distances, recalculateFlag)
    if (recalculateFlag)
        fyData = []; %zeros(distances.size(), 
        GIGADATA = [];
        POINT_COUNT = [];
        for cur_model = modelDict
            disp(cur_model)
            [fyData, counts] = computePlaneErrorDistances(distances, stereoParams, cur_model)
            disp("Model ready")
            GIGADATA = [GIGADATA fyData.'] 
            POINT_COUNT = [POINT_COUNT counts.']
        end
    end
    
    createfigure(distances, POINT_COUNT);
    createfigure(distances, GIGADATA);
end

function [mses, counts] = computePlaneErrorDistances(distances, stereoParams, modelName)
    mses = zeros(size(distances), 'double');    
    counts = zeros(size(distances), 'double'); 
    order = int16(1);
    for dst = distances
        %%%     FISHEYE        %%%
        [fy_e, fy_disp, fy_mean, count] = computePlaneError(stereoParams, dst, modelName);
%         fy_count = [fy_count count];
        mses(int16(order)) =  fy_e;
        counts(int16(order)) = count;
        disp("Distance " + dst + " ready")
        order = order +1;
    end
end

function [MSE, D, M, Inds]  = computePlaneError(stereoParams, distance, type)
    global  BasePath;
    global SHOW;
    global Scale;
    %base_path = BasePath + string(distance) + "m\";  % compar0.3m

    targetDistance = distance;
%     target_roi = [-0.4 0.6 -0.6 0.46 0.90*targetDistance/Scale 1.1*targetDistance/Scale];       % plane is a little shifted
    k = distance / 0.7;
    target_roi = [-0.4*k 0.6*k -0.6*k 0.46*k 0.50*targetDistance/Scale 1.5*targetDistance/Scale];

    targetParamsVector = [0, 0, 1, -targetDistance/Scale];   % normal + distance
    ref_model = planeModel(targetParamsVector);

    imgLeft = BasePath + type + "/l_img_"+ type + string(distance-1) + ".png";
    imgRight = BasePath + type + "/r_img_" + type + string(distance-1) + ".png";
%     if (type~="REG")          % in older datasets L/R were mistaken                           yikes
%         imgRight = BasePath + type + "/l_img_"+ type + string(distance-1) + ".png";
%         imgLeft = BasePath + type + "/r_img_" + type + string(distance-1) + ".png";
%     end

    lImage = imread(imgLeft);
    rImage = imread(imgRight);

    [frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, stereoParams);

    %figure;
    %imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
    %title('Rectified Video Frames');

    frameLeftGray  = rgb2gray(frameLeftRect);
    frameRightGray = rgb2gray(frameRightRect);
    
    disparityMapReg = disparitySGM(frameLeftGray, frameRightGray);          %disparityBM   disparitySGM
   

    points3Dreg = reconstructScene(disparityMapReg, stereoParams);
    points3Dreg = points3Dreg ./ 1000;
    ptCloud = pointCloud(points3Dreg, 'Color', frameLeftRect);
    if (SHOW) 
        figure('Name',type+' full_scene'+string(targetDistance))
        pcshow(ptCloud);
    end
    indicies = findPointsInROI(ptCloud, target_roi);
    ptCloud = select(ptCloud, indicies);
    Inds = ptCloud.Count;
    [MSE, D] = findMSE(ptCloud,  ref_model);
%     MSE = 0;
%     D  = 0;
    M = mean(ptCloud.Location(:,3))*Scale;
    
    if (SHOW)    
        disp(type+"ERROR: ")
        disp(MSE)
        % Visualize the point cloud
        figure('Name',type+' depth'+string(targetDistance))
        pcshow(ptCloud);
        hold on
        plot(ref_model, 'color', 'white')
        hold off
        
%         figure;
%         imshow(disparityMapReg, [0, 64]);
%         title('Disparity Map'+string(targetDistance));
%         colormap jet
%         colorbar
    end
end

function [MSE, D] = findMSE(pt_cloud, plane_model)
    global Scale;
    error_sum = 0.0;
    squaredError_sum = 0.0;
    ortho_method = false;
%    figure;
%     Animated_Plot = animatedline;
    z_dst = -plane_model.Parameters(4);
    size = (10/Scale * z_dst / 0.7)/2;            % half side length 
    edge_tan = size / z_dst;

    for elm = 1:pt_cloud.Count
        if (ortho_method)
            pnt = pt_cloud.Location(elm,:);
            planePoint = [0, 0, z_dst];  % plane center
            PQ = pnt - planePoint;
            error = dot(PQ, plane_model.Normal)*Scale;     % difference [meters] 
            error_sum = error_sum + error;
            squaredError_sum = squaredError_sum + error^2;
        else
            pnt = pt_cloud.Location(elm,:);
            if (abs(pnt(1)/pnt(3)) < edge_tan || abs(pnt(2)/pnt(3)) < edge_tan)
                planePoint = [0, 0, z_dst];  % plane center
                pnt_dist = norm(pnt);
                plane_dist = pnt_dist * z_dst / pnt(3);
                error = (plane_dist - pnt_dist)*Scale;
                error_sum = error_sum + error;
                squaredError_sum = squaredError_sum + error^2;                
            end
        end

        
    end
    
    MSE = sqrt(squaredError_sum/pt_cloud.Count);
    D = squaredError_sum/pt_cloud.Count;
    
end

function createfigure(X1, YMatrix1)
%CREATEFIGURE(X1, YMatrix1)
%  X1:  vector of x data
%  YMATRIX1:  matrix of y data

%  Auto-generated by MATLAB on 10-Apr-2022 23:59:03

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple lines using matrix input to plot
plot1 = plot(X1,YMatrix1,'LineWidth',2,'Parent',axes1);
set(plot1(1),'DisplayName','REG');
set(plot1(2),'DisplayName','ATAN','Marker','o','LineStyle','--');
set(plot1(3),'DisplayName','MEI','Marker','square','LineStyle',':');
set(plot1(4),'DisplayName','SCARA','Marker','diamond','LineStyle','-.');
set(plot1(5),'DisplayName','KB','LineStyle','--');
set(plot1(6),'DisplayName',['REAL',newline,'ATAN'],'LineWidth',3);

% Create ylabel
ylabel('MSE, m');

% Create xlabel
xlabel('Distance, m');

% Create title
title('Models');

box(axes1,'on');
% Set the remaining axes properties
set(axes1,'XGrid','on','YGrid','on');
% Create legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.776501766784452 0.145977011494253 0.0990065978549978 0.178347160479719]);

end
