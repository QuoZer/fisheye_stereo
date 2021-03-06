close all;
global SHOW BasePath Scale RECALCULATE Models ParamDict;

RECALCULATE = true;
SHOW = false;
BasePath = "D:\Work\Coding\Repos\fisheye_stereo\data\1080p[REAL]\1_Compar0.1m\"; %+ Model + "\";  %0.1m\
Scale = 1; 
Models =["SCARA" "KB" "MEI" "DS"] % "REG" "ATAN"  "REAL_ATAN" 
ParamDict = {stereoParamsSCARA, stereoParamsKB stereoParamsMEI stereoParamsDS};
distances = [1 2 3 4 5 6 ]; % 0.5m: 4 5 6 7.5 9 10 11 12 12.5 13 14 15  //  0.3m: 1 2 4 5 6 7.5 10 // 0.05m: 1 2 3 4 5 6 8 10 12
% MEI0.1: 1 2 3 4 5 6 7 8
% ALL 1 2 3 4 5 6 7 8 9 

computeMeanMSA(BasePath,3, distances, stereoParamsSCARA )
% diffImageAll(Models, 7);

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
        
        pout_imadjust = imadjust(diffImage, [0.1 0.3]);
%         pout_histeq = histeq(diffImage);
%         pout_adapthisteq = adapthisteq(diffImage);
        
%         montage({diffImage,pout_imadjust,pout_histeq,pout_adapthisteq},"Size",[1 4])
%         title("Original Image and Enhanced Images using imadjust, histeq, and adapthisteq")
        
        figure('Name', 'Diff Image '+type);
        imshow(pout_imadjust)
    end
end

function computeMeanMSA(dataset_path, num_of_folders, distances, stereoparams)
    global RECALCULATE;
    global Models;
    global  BasePath;
    global MSES_SUM
    global COUNT_SUM;
    global MEAN_SUM;
    global DISP_SUM;
    global ALIGN_SUM;
    
    % EXPORT THEM
    if (RECALCULATE)
        MSES_SUM  = zeros(size(distances, 2), size(Models, 2));
        COUNT_SUM = zeros(size(distances, 2), size(Models, 2));
        DISP_SUM  = zeros(size(distances, 2), size(Models, 2)*3);
        MEAN_SUM  = zeros(size(distances, 2), size(Models, 2));
        ALIGN_SUM = zeros(size(distances, 2), size(Models, 2));
        for ind = 1:num_of_folders
            BasePath = "D:\Work\Coding\Repos\fisheye_stereo\data\1080p[REAL]\" + string(ind) + "_Compar0.072m\";  %540p[ALL RIGHT]\
            [mses, counts, disps, means, align] = computeAllModels(Models, stereoparams, distances, RECALCULATE);
            MSES_SUM = MSES_SUM + mses;
            DISP_SUM = DISP_SUM + disps;
            COUNT_SUM = COUNT_SUM + counts;
            MEAN_SUM = MEAN_SUM + means;
            ALIGN_SUM = ALIGN_SUM + align;

        end
        COUNT_SUM = COUNT_SUM/num_of_folders;
        MSES_SUM = MSES_SUM/num_of_folders;
        COUNT_SUM = COUNT_SUM/num_of_folders;
        MEAN_SUM = MEAN_SUM/num_of_folders;
        ALIGN_SUM = ALIGN_SUM/num_of_folders;
    end
    
    createfigureReal(0.25*distances, MSES_SUM, '???');
    createfigureReal(0.25*distances, COUNT_SUM, '????? ?????');
    createfigureReal(0.25*distances, ALIGN_SUM, '????????');
    MseAndDispReal(0.25*distances, MEAN_SUM, DISP_SUM);
end

function [MSE_DATA, POINT_COUNT, DISP_DATA, MEAN_DATA, ALIGN_DATA] = computeAllModels(modelDict, stereoParams, distances, recalculateFlag)
    global ParamDict;
    if (recalculateFlag)
        fyData = []; %zeros(distances.size(), 
        MSE_DATA = [];
        POINT_COUNT = [];
        DISP_DATA = [];
        MEAN_DATA = [];
        ALIGN_DATA = [];
        
        for cur_m_index = 1:size(modelDict, 2)
            cur_model = modelDict(cur_m_index);
            cur_params = ParamDict{cur_m_index};
            disp(cur_model)
            
            
            [fyData, counts, disps, means, misalign] = computePlaneErrorDistances(distances, cur_params, cur_model);
            disp("Model ready")
            MSE_DATA = [MSE_DATA fyData.'] ;
            POINT_COUNT = [POINT_COUNT counts.'];
            DISP_DATA = [DISP_DATA disps.'];
            MEAN_DATA = [MEAN_DATA means.'];
            ALIGN_DATA = [ALIGN_DATA misalign.'];
        end
    end
    
%     createfigure(distances, POINT_COUNT);
%     createfigure(distances, MSE_DATA);
end

function [mses, counts, disps, means, misalignment] = computePlaneErrorDistances(distances, stereoParams, modelName)
    mses = zeros(size(distances), 'double');    
    counts = zeros(size(distances), 'double'); 
    disps = zeros(3,size(distances, 2),  'double');    
    means = zeros(size(distances), 'double');
    misalignment = zeros(size(distances), 'double');
    
    order = int16(1);
    for dst = distances
        %%%     FISHEYE        %%%
        [fy_e, fy_disp, fy_mean, count, misalign] = computePlaneError(stereoParams, dst, modelName);
%         fy_count = [fy_count count];
        mses(int16(order)) =  fy_e;
        counts(int16(order)) = count;
        disps(:,int16(order)) =fy_disp;
        means(int16(order)) =fy_mean;
        misalignment(int16(order)) = misalign;
        
        disp("Distance " + dst + " ready")
        order = order +1;
    end
end

function [MSE, D, M, Inds, misalign]  = computePlaneError(stereoParams, distance, type)
    close all;    
    global  BasePath;
    global SHOW;
    global Scale;
    %base_path = BasePath + string(distance) + "m\";  % compar0.3m

    targetDistance = distance*0.25;
    target_roi = [-0.06 0.16 -0.19 0.018 0.70*targetDistance 1.3*targetDistance];       % plane is a little shifted
%     k = distance / 0.7;
%     target_roi = [-0.5*k 0.5*k -0.5*k 0.5*k 0.85*targetDistance/Scale 1.15*targetDistance/Scale];

    targetParamsVector = [0, 0, 1, -targetDistance/Scale];   % normal + distance
    ref_model = planeModel(targetParamsVector);

    imgLeft = BasePath + type + "/l_img_"+ type + string(distance-1) + ".png";
    imgRight = BasePath + type + "/r_img_" + type + string(distance-1) + ".png";
%     if (type=="DS")          % in older datasets L/R were mistaken                           yikes
%         imgRight = BasePath + type + "/l_img_"+ type + string(distance-1) + ".png";
%         imgLeft = BasePath + type + "/r_img_" + type + string(distance-1) + ".png";
%     end

    lImage = imread(imgLeft);
    rImage = imread(imgRight);

    [frameLeftRect, frameRightRect] = rectifyStereoImages(lImage, rImage, stereoParams);

    frameLeftGray  = rgb2gray(frameLeftRect);
    frameRightGray = rgb2gray(frameRightRect);
    
    disparityMapReg = disparitySGM(frameLeftGray, frameRightGray);          %disparityBM   disparitySGM
   %% PTC processing
    points3Dreg = reconstructScene(disparityMapReg, stereoParams);
    points3Dreg = points3Dreg ./ 1000;
    ptCloud = pointCloud(points3Dreg, 'Color', frameLeftRect);
    if (SHOW) 
        figure('Name',type+' full_scene'+string(targetDistance))
        pcshow(ptCloud);
    end
    indicies = findPointsInROI(ptCloud, target_roi);
    ptCloud = select(ptCloud, indicies);
    % how many points to leave in ptc
    percent_to_stay = 1000/ptCloud.Count;
    if (percent_to_stay > 1)
        percent_to_stay = 1;
    end
    ptCloud = pcdownsample(ptCloud, 'random', percent_to_stay);
    pcshow(ptCloud);
    [new_vec, TF] = rmoutliers(ptCloud.Location(:,3));
    ptCloud = select(ptCloud, ~TF);
    pcshow(ptCloud);
    Inds = ptCloud.Count;
    %% Stats
    [MSE, D, M] = findMSE_D_M(ptCloud,  ref_model);
    misalign = computeAlignment(ptCloud, distance);
   %% Display
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
    %%
end

function misalignment = computeAlignment(pt_cloud, distance)
    maxDistance = 0.05*distance;
    referenceVector = [0,0,1];
    maxAngularDistance = 15;
    
    [model2,inlierIndices,outlierIndices] = pcfitplane(pt_cloud,...
            maxDistance,referenceVector,maxAngularDistance); 
    second_vector = model2.Normal;
    
    misalignment = 1 - dot(second_vector, referenceVector);
end

function [MSE, D, M] = findMSE_D_M(pt_cloud, plane_model)
    global Scale;
    error_sum = 0.0;
    squaredError_sum = 0.0;
    ortho_method = true;
    smallest_D = 1000;
    biggest_D = 0;
    z_dst = -plane_model.Parameters(4);
    size = (10/Scale * z_dst / 0.7)/2;            % half side length 
    edge_tan = size / z_dst;

    for elm = 1:pt_cloud.Count
        pnt = pt_cloud.Location(elm,:);
        if (pnt(3) > biggest_D)
            biggest_D = pnt(3);
        end
        if (pnt(3) < smallest_D)
            smallest_D = pnt(3);
        end
        
        if (ortho_method)
            planePoint = [0, 0, z_dst];  % plane center
            PQ = pnt - planePoint;
            error = dot(PQ, plane_model.Normal)*Scale;     % difference [meters] 
            error_sum = error_sum + error;
            squaredError_sum = squaredError_sum + error^2;
        else
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
    M = mean(pt_cloud.Location(:,3))*Scale;
    D = [squaredError_sum/pt_cloud.Count; M-smallest_D; biggest_D-M];
    MSE = sqrt(D(1));
end

function createfigureVirt(X1, YMatrix1)
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
set(plot1(7),'DisplayName','DS','LineWidth',3);

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

function createfigureReal(X1, YMatrix1, Title)
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
set(plot1(3),'DisplayName','MEI','Marker','square','LineStyle',':');
set(plot1(2),'DisplayName','SCARA','Marker','diamond','LineStyle','-.');
set(plot1(1),'DisplayName','KB','LineStyle','--');
set(plot1(4),'DisplayName','DS','LineWidth',3);

% Create ylabel
ylabel('MSE, m');

% Create xlabel
xlabel('Distance, m');

% Create title
title(Title);

box(axes1,'on');
% Set the remaining axes properties
set(axes1,'XGrid','on','YGrid','on');
% Create legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.776501766784452 0.145977011494253 0.0990065978549978 0.178347160479719]);

end

function MseAndDispVirt(XMatrix1, YMatrix1, DMatrix1)
%CREATEFIGURE(XMatrix1, YMatrix1, DMatrix1)
%  XMATRIX1:  errorbar x matrix data
%  YMATRIX1:  errorbar y matrix data
%  DMATRIX1:  errorbar delta matrix data

%  Auto-generated by MATLAB on 22-Apr-2022 01:29:06

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple error bars using matrix input to errorbar
% "REG" "ATAN" "MEI" "SCARA" "KB" "REAL_ATAN" "DS"
errorbar1 = errorbar(XMatrix1,YMatrix1(:,1),DMatrix1(:,1),'LineWidth',2);
errorbar2 = errorbar(XMatrix1,YMatrix1(:,2),DMatrix1(:,2),'LineWidth',2);
errorbar3 = errorbar(XMatrix1,YMatrix1(:,3),DMatrix1(:,3),'LineWidth',2);
errorbar4 = errorbar(XMatrix1,YMatrix1(:,4),DMatrix1(:,4),'LineWidth',2);
errorbar5 = errorbar(XMatrix1,YMatrix1(:,5),DMatrix1(:,5),'LineWidth',2);
errorbar6 = errorbar(XMatrix1,YMatrix1(:,6),DMatrix1(:,6),'LineWidth',2);
errorbar7 = errorbar(XMatrix1,YMatrix1(:,7),DMatrix1(:,7),'LineWidth',2);
set(errorbar1,'DisplayName','REG');
set(errorbar2,'DisplayName','ATAN','LineStyle','-.');
set(errorbar3,'DisplayName','MEI','Marker','diamond','LineStyle','-.');
set(errorbar4,'DisplayName','SCARA','Marker','square','LineStyle',':');
set(errorbar5,'DisplayName','KB','Marker','o','LineStyle','--');
set(errorbar6,'DisplayName','REAL_ATAN','Marker','o','LineStyle','--');
set(errorbar7,'DisplayName','DS','Marker','diamond','LineStyle',':');

% Create ylabel
ylabel('???, ?');

% Create xlabel
xlabel('??????????, ?');

% Create title
title('Mean MSE&D ');

legend1 = legend(axes1,'show');

% Uncomment the following line to preserve the X-limits of the axes
% xlim(axes1,[0.84938193320436 10.1698459049717]);
% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes1,[-0.0970957959651191 1.94176474770099]);
% Set the remaining axes properties
set(axes1,'LineStyleOrder',{'- +'},'XGrid','on','YGrid','on');
end

function MseAndDispReal(XMatrix1, YMatrix1, DMatrix1)
%CREATEFIGURE(XMatrix1, YMatrix1, DMatrix1)
%  XMATRIX1:  errorbar x matrix data
%  YMATRIX1:  errorbar y matrix data
%  DMATRIX1:  errorbar delta matrix data

%  Auto-generated by MATLAB on 22-Apr-2022 01:29:06

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple error bars using matrix input to errorbar
% "KB" "SCARA" "MEI" "DS"
errorbar1 = errorbar(XMatrix1,YMatrix1(:,1),DMatrix1(:,2),DMatrix1(:,3),'LineWidth',2);
errorbar2 = errorbar(XMatrix1,YMatrix1(:,2),DMatrix1(:,5),DMatrix1(:,6),'LineWidth',2);
errorbar3 = errorbar(XMatrix1,YMatrix1(:,3),DMatrix1(:,8),DMatrix1(:,9),'LineWidth',2);
errorbar4 = errorbar(XMatrix1,YMatrix1(:,4),DMatrix1(:,11),DMatrix1(:,12),'LineWidth',2);
set(errorbar3,'DisplayName','MEI','Marker','diamond','LineStyle','-.');
set(errorbar2,'DisplayName','SCARA','Marker','square','LineStyle',':');
set(errorbar1,'DisplayName','KB','Marker','o','LineStyle','--');
set(errorbar4,'DisplayName','DS','Marker','diamond','LineStyle',':');

% Create ylabel
ylabel('???, ?');

% Create xlabel
xlabel('??????????, ?');

% Create title
title('Mean MSE&D ');

legend1 = legend(axes1,'show');

% Uncomment the following line to preserve the X-limits of the axes
% xlim(axes1,[0.84938193320436 10.1698459049717]);
% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes1,[-0.0970957959651191 1.94176474770099]);
% Set the remaining axes properties
set(axes1,'LineStyleOrder',{'- +'},'XGrid','on','YGrid','on');
end