%%
clear; clc; close all;
testMode = 'training'; seqName_ = 'junction'; seqName = 'junction'; seqNb = '0000';
% addpath(genpath('/home/jiang/CvTools/planeFit/affine_fit'));
dataDir = sprintf('/home/jiang/catkin_ws/results/kitti_%s_regist/', seqName);
registCloud_files = dir([dataDir, '*.pcd']);cloud_all = [];
dataGndDir = sprintf('/mnt/47dd6768-cdcb-489e-8f25-5087db1db873/CVPR17/kittiObjectGndPlane/%s/', seqName);
gndCloud_files = dir([dataGndDir, '*_Gnd.pcd']);
noGndCloud_files = dir([dataGndDir, '*_NoGnd.pcd']);%dir([dataGndDir, '*_NoGnd.pcd']);
paramCfg.calib_dir = ['/home/jiang/CvDataset/KITTI/tracking_module/',testMode,'/calib/'];
paramCfg.imgDir    = ['/home/jiang/CvDataset/KITTI/tracking_module/',testMode,'/image_02/', seqNb,'/'];
binDir = sprintf('/home/jiang/CvDataset/KITTI/tracking_module/training/velodyne/%s/', seqNb);
calibFile = [ paramCfg.calib_dir,sprintf('%s.txt', seqNb)];
pose_aft_mapped_init = load([dataDir, 'pose_intg_to_init.txt']);
saveDir = sprintf('/mnt/47dd6768-cdcb-489e-8f25-5087db1db873/CVPR17/kittiMOD/object_%s/', seqNb);

% Camera pose estimated from Lidar Odometry And Mapping (LOAM)
pose_LOAM = pose_aft_mapped_init;
data3D = []; fullMap = []; fullMapColor = [];
% load('MOD_Qualitative_market_10_1020.mat');
% load('MOD_Qualitative_junction_10_150.mat');
%% Setting for frame to be processed
params.frmSta = 1;   % starting frame
params.frmEnd = 150; % end frame
procfrm = 1; % frame interval, default process all the frames
%% Static-map and Camera Poses
staticMapDir = '/mnt/47dd6768-cdcb-489e-8f25-5087db1db873/CVPR17/kittiMOD/object_0019/regist/';
staticMapDir = '/mnt/47dd6768-cdcb-489e-8f25-5087db1db873/CVPR17/kittiMOD/object_0000/StaticPcd/';
staticMapFile = dir([staticMapDir, '*_static_*.ply']);
staticSegIdx = 1;
useKittiObject = 1;
shiftIdx = 10;
colors = distinguishable_colors(7);
for cIdx = params.frmSta:procfrm:params.frmEnd
    cIdx
    %% Load the static maps
    % You can select N frames of pointcloud to generate synthestic images
    % with minimum 100 frames
    if 0
        if cIdx==params.frmSta || mod(cIdx, 100) == 0 && staticSegIdx+5 <= length(staticMapFile)
            staticMap = []; staticMapColor = [];
            for iStatic = staticSegIdx:min(staticSegIdx+5, length(staticMapFile))
                staticMapTmp = pcread([staticMapDir, staticMapFile(iStatic).name]);
                staticMap = [staticMap; staticMapTmp.Location];
                staticMapColor = [staticMapColor;staticMapTmp.Color];
            end
            staticMap = pointCloud(staticMap, 'Color',staticMapColor);
            figure(5001); showPointCloud(staticMap);
            staticSegIdx = staticSegIdx+1;
        end
        staticMapXYZ = staticMap.Location;
        staticMapColor = staticMap.Color;
    end
    cloudNoGnd = loadpcd([dataGndDir, ...
        noGndCloud_files(cIdx).name]);
    cloudFoV = loadpcd([dataDir, ...
        registCloud_files(cIdx).name]);
    
    %% load calibration
    paramCfg.P_velo_to_img = load_kitti_calibration(calibFile, useKittiObject);
    
    %% Project 3D data to image
    % First, transform the static map into camera coordinate using LOAM poses.
    cloudRegist = loadpcd([dataDir, registCloud_files(cIdx).name]);
    fName = [binDir, sprintf('%06d.bin', cIdx+shiftIdx)];
    fid  = fopen(fName,'rb');
    veloOrg = fread(fid,[4 inf],'single')';
    fclose(fid);
    convertCoordinate = ([0 0 1 0; 1, 0, 0 0; 0 1 0 0; 0 0 0 1]);
    Tr_xyz = pose_LOAM(cIdx,2:4);
    quat = pose_LOAM(cIdx,5:8);
    RotMat = quat2rotm( [quat(4), quat(1:3)]);
    poseTrans = [RotMat, Tr_xyz(1:3)';0, 0, 0, 1];
    Tmapping = convertCoordinate*poseTrans*pinv(convertCoordinate);
    
    % Second, refine the pose of the static map with current frame (3D cloud)
    % using ICP registration.
    velo3d = cloudFoV;
    cloudRegist = pinv(Tmapping)*[cloudRegist; ones(1, size(cloudRegist,2))];
    cloudNoGnd = pinv(Tmapping)*[cloudNoGnd; ones(1, size(cloudNoGnd,2))];
    velo3d = pinv(Tmapping)*[velo3d; ones(1, size(velo3d,2))];
    [tform,velo3dReg] = pcregrigid(pointCloud(cloudRegist(1:3,:)'), pointCloud(veloOrg(:, 1:3)), 'Tolerance', [0.000001, 0.0000009], 'InlierRatio',1.0);
    velo3dReg = (tform.T')*velo3d;
    velo3dReg = velo3dReg(1:3, :)';
    velo3d = cloudFoV';
    maxX = max( veloOrg(:, 1) );
    cloudNoGnd = (tform.T')*cloudNoGnd;
    cloudNoGnd = cloudNoGnd(1:3, :)';
    
    %         figure(1003),clf; showPointCloud(staticMapXYZ(:, 1:3), staticMap.Color); hold on;
    %         showPointCloud(velo3dReg, [0 1 0]);
    %         showPointCloud(velo3d_(1:3,:)', [0 0 1]);
    leftImgRef = imread([paramCfg.imgDir, sprintf('/%06d.png', cIdx+shiftIdx)]);
    [velo3dRegFoV, velo3dRegFoV_Color, fovProj3DUnique] =...
        associate2d3d_kitti(leftImgRef, velo3dReg(:, 1:3), paramCfg.P_velo_to_img);
    
    [cloudNoGndFoV, cloudNoGndFoV_Color, cloudNoGndProj3DUnique] =...
        associate2d3d_kitti(leftImgRef, cloudNoGnd(:, 1:3), paramCfg.P_velo_to_img);
    
    %     figure(1000),clf; pcshow(velo3dRegFoV, uint8(velo3dRegFoV_Color)); drawnow;
    %     figure(1001), imshow(leftImgRef); hold on; plot(fovProj3DUnique(1,:), fovProj3DUnique(2,:), '.g');
    %     imwrite(projStaticImg, ['/mnt/47dd6768-cdcb-489e-8f25-5087db1db873/CVPR17/kittiMOD/object_0019/registImg',...
    %         sprintf('/%06d.png', cIdx+10)]);
    %     figure(1002), pcshow(cloudNoGndFoV, uint8(cloudNoGndFoV_Color)); drawnow;
    
    %% Load Object labels
    
    labelFile = sprintf('%s%06d_yolo.txt', paramCfg.imgDir,cIdx+shiftIdx);
    bbox = []; objLabels = []; objLabels2 = [];
    [bbox(:,1), bbox(:,2), bbox(:,3), bbox(:,4), objLabels, objLabels2] = ...
        textread(labelFile,'%d %d %d %d %s %s');
    
    velo3dRegMap = Tmapping*pinv(tform.T')*[velo3dRegFoV'; ones(1, size(velo3dRegFoV,1))];
    fullMap = [fullMap;velo3dRegMap(1:3,:)'];
    fullMapColor = [fullMapColor; uint8(velo3dRegFoV_Color)];
    figure(1000); pcshow(velo3dRegMap(1:3,:)', uint8(velo3dRegFoV_Color)); drawnow;
    figure(1001), imshow(leftImgRef);
    for lidx = 1:size(bbox,1)
        objIdx_x = intersect(find(cloudNoGndProj3DUnique(1, :)>bbox(lidx, 1)),...
            find(cloudNoGndProj3DUnique(1, :)<bbox(lidx, 3)));
        objIdx_y = intersect(find(cloudNoGndProj3DUnique(2, :)>bbox(lidx, 2)),...
            find(cloudNoGndProj3DUnique(2, :)<bbox(lidx, 4)));
        objIdx = intersect(objIdx_x, objIdx_y);
        bbCloud = cloudNoGndFoV(objIdx,:);
        bbCloud = sortrows(bbCloud, 3);
        bbCloudNb = floor(size(bbCloud, 1)/2);
        madXYZ = [median(bbCloud(1:bbCloudNb,1)),...
            median(bbCloud(1:bbCloudNb,2)),...
            median(bbCloud(1:bbCloudNb,3))];
        for bloop = 1:2
            if strcmp(objLabels{lidx}, 'car')
                box3d = [madXYZ + [3.5, 3.5, 2], madXYZ - [3.5, 3.5, 2]];
            else
                box3d = [madXYZ + [1.5, 1.5, 1], madXYZ - [1.5, 1.5, 1]];
            end
            bbIdx_x = intersect(find(bbCloud(:,1)<box3d(1)), ...
                find(bbCloud(:,1)>box3d(4)));
            bbIdx_y = intersect(find(bbCloud(:,2)<box3d(2)), ...
                find(bbCloud(:,2)>box3d(5)));
            bbIdx_z = intersect(find(bbCloud(:,3)<box3d(3)), ...
                find(bbCloud(:,3)>box3d(6)));
            bbIdx = intersect(bbIdx_x, bbIdx_y);
            bbIdx = intersect(bbIdx, bbIdx_z);
            bbCloud = bbCloud(bbIdx, :);
            if isempty(bbCloud) | size(bbCloud, 1)<3
                continue;
            end
            [bboxOriented, mu, bbAxis] = fitBBoxHorizontal(bbCloud);      % pca
            madXYZ = mu;
        end
        if  isempty(bbCloud)
            continue;
        end
        bbCloud_img = preciseObjectLocalization(velo3dRegFoV, bboxOriented, paramCfg.P_velo_to_img, bbAxis);
        bboxOriented = Tmapping*pinv(tform.T')*[bboxOriented'; ones(1, 8)];
        bboxOriented = bboxOriented(1:3, :)';
        colorDraw = [];
        if strcmp(objLabels{lidx}, 'car')
            colorDraw = colors(3,:);
        elseif strcmp(objLabels{lidx}, 'person')
            colorDraw = colors(2,:);
        elseif strcmp(objLabels{lidx}, 'bicycle')
            colorDraw = colors(1,:);
        elseif strcmp(objLabels{lidx}, 'motorbike')
            colorDraw = colors(4,:);
        elseif strcmp(objLabels{lidx}, 'bus')
            colorDraw = colors(5,:);
        elseif strcmp(objLabels{lidx}, 'traffic')
            colorDraw = colors(6,:);
        else
            colorDraw = colors(7,:);
        end
        figure(1000), hold on; plotCube(bboxOriented, colorDraw); hold off;
        figure(1001); hold on;
        %         plot(bbCloud_img(:,1),bbCloud_img(:,2),'s','Color', colorDraw);
        k = boundary(bbCloud_img(:,1),bbCloud_img(:,2));
        plot(bbCloud_img(k,1),bbCloud_img(k,2), '.-','Color', colorDraw, 'LineWidth', 3);
        hold off;
    end
    
        bbox(:, 3) = bbox(:, 3) - bbox(:, 1);
        bbox(:, 4) = bbox(:, 4) - bbox(:, 2);
        for lidx = 1:size(bbox,1)
            figure(1001); hold on;
            rectangle('Position', bbox(lidx,:),...
            'EdgeColor','r', 'LineWidth', 0.5);
        end
    saveDir = './label3d/';
    if ~exist(saveDir)
        mkdir(saveDir);
    end
    %     saveas(gcf, sprintf('%s%06d_3dBoxTraj.png',saveDir,cIdx+shiftIdx));

end
