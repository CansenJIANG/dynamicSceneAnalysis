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

addpath(genpath('/home/jiang/CvTools/deepmatching_1.2.2_c++'))
addpath(genpath('/home/jiang/CvTools/flow-code-matlab'))
% Camera pose estimated from Lidar Odometry And Mapping (LOAM)
pose_LOAM = pose_aft_mapped_init;
data3D = []; fullMap = []; fullMapColor = [];
% load('MOD_Qualitative_market_10_1020.mat');
% load('MOD_Qualitative_junction_10_150.mat');
%% Setting for frame to be processed
params.frmSta = 1;   % starting frame
params.frmEnd = 140; % end frame
procfrm = 1; % frame interval, default process all the frames
%% Static-map and Camera Poses
staticMapDir = '/mnt/47dd6768-cdcb-489e-8f25-5087db1db873/CVPR17/kittiMOD/object_0019/regist/';
staticMapDir = '/mnt/47dd6768-cdcb-489e-8f25-5087db1db873/CVPR17/kittiMOD/object_0000/StaticPcd/';
staticMapFile = dir([staticMapDir, '*_static_*.ply']);
staticSegIdx = 1;
useKittiObject = 1;
shiftIdx = 10;
colors = distinguishable_colors(7);
objectList = [];
%objectBoxLbls = []; movingObjIdx = []; objectTracker = [];
% load('cIdx93.mat');
for cIdx = params.frmSta:procfrm:params.frmEnd
    cIdx
    if cIdx == 91
        cIdx
    end
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
    
    %% Compute 3D - 3D correspondences
    [noGndProjT0, noGndRegistT0, noGndProjT1, noGndRegistT1] = ...
        deepFlowTracking2(paramCfg.imgDir, binDir, dataGndDir,...
        dataDir, cIdx, shiftIdx, paramCfg.P_velo_to_img, seqNb, Tmapping);
    
    %% Load Object labels
    
    labelFile = sprintf('%s%06d_yolo.txt', paramCfg.imgDir,cIdx+shiftIdx);
    bbox = []; objLabels = []; prob = [];
    [bbox(:,1), bbox(:,2), bbox(:,3), bbox(:,4), prob, objLabels] = ...
        textread(labelFile,'%d %d %d %d %f %s');
    
    velo3dRegMap = Tmapping*pinv(tform.T')*[velo3dRegFoV'; ones(1, size(velo3dRegFoV,1))];
    
    figure(1000); hold on; showPointCloud([0 0 0]); % 
    pcshow(velo3dRegMap(1:3,:)', uint8(velo3dRegFoV_Color), 'MarkerSize', 15); drawnow;
    figure(1001), imshow(leftImgRef);
    drewBoxList = [];
    for lidx = 1:size(bbox,1)
        objIdx_x = intersect(find(cloudNoGndProj3DUnique(1, :)>bbox(lidx, 1)),...
            find(cloudNoGndProj3DUnique(1, :)<bbox(lidx, 3)));
        objIdx_y = intersect(find(cloudNoGndProj3DUnique(2, :)>bbox(lidx, 2)),...
            find(cloudNoGndProj3DUnique(2, :)<bbox(lidx, 4)));
        objIdx = intersect(objIdx_x, objIdx_y);
        if isempty(objIdx)
            continue;
        end
        bbCloud = cloudNoGndFoV(objIdx,:);
        bbCloud = sortrows(bbCloud, 3);
        bbCloudNb = floor(size(bbCloud, 1));
        madXYZ = [median(bbCloud(0.2*bbCloudNb:0.8*bbCloudNb,1)),...
            median(bbCloud(1:bbCloudNb,2)),...
            median(bbCloud(1:bbCloudNb,3))];
        for bloop = 1:2
            if strcmp(objLabels{lidx}, 'car')
                box3d = [madXYZ + [3.5, 3.5, 3], madXYZ - [3.5, 3.5, 3]];
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
            [bboxCam, mu, bbAxis] = fitBBoxHorizontal(bbCloud);      % bounding box in camera frame
            madXYZ = mu;
        end
        if  isempty(bbCloud)
            continue;
        end
        [bbCloud_img, objCloud] = preciseObjectLocalization(velo3dRegFoV, bboxCam, paramCfg.P_velo_to_img, bbAxis);
        bboxMap = Tmapping*pinv(tform.T')*[bboxCam'; ones(1, 8)]; % bounding box in map frame
        bboxMap = bboxMap(1:3, :)';
        
        % varify object existance
        addNewBox = 1;
        
        if length(objectList) > 0
            for ibox = 1:length(objectList)
                boxTmp = objectList{ibox}.bboxMap;
                boxNew = bboxMap;
                cenBoxTmp = mean(boxTmp);
                maxDistTmp = norm(cenBoxTmp - boxTmp(1,:));
                cenBoxNew = mean(boxNew);
                maxDistNew = norm(cenBoxNew - boxNew(1,:));
                distBoxTmpNew = norm(cenBoxTmp - cenBoxNew);
                if (distBoxTmpNew < max(maxDistTmp, maxDistNew) &&...
                        max(maxDistTmp, maxDistNew)/distBoxTmpNew > 0.1)
                    addNewBox = 0;
                    if objectList{ibox}.frameNb ~= cIdx && isempty(find(drewBoxList==ibox))
                        newbbox2d = [bbox(lidx, 1:2), bbox(lidx, 3:4)-bbox(lidx, 1:2)];
                        drawObjectInformation(objectList{ibox}, colors, newbbox2d);
                        drewBoxList = [drewBoxList, ibox];
                    end
                end
            end
        end
        if addNewBox
            volBox = norm(bboxMap(1,:)-bboxMap(2,:))*...
                norm(bboxMap(1,:)-bboxMap(3,:))*...
                norm(bboxMap(1,:)-bboxMap(4,:));
            if  strcmp(objLabels{lidx}, 'car') && (volBox > 30)
                addNewBox = 0;
            elseif strcmp(objLabels{lidx}, 'bus') && (volBox > 80)
                addNewBox = 0;
            elseif strcmp(objLabels{lidx}, 'person') && (volBox > 3)
                addNewBox = 0;
            elseif strcmp(objLabels{lidx}, 'bicycle') && (volBox > 3)
                addNewBox = 0;
            end
        end
        addNewBox = 1;
        if addNewBox 
            objParam = [];
            objParam.bboxCam = bboxCam;
            objParam.bboxMap = bboxMap;
            objParam.bbAxis = bbAxis;
            objParam.size = get3dBBoxVolumn(bboxMap);
            objParam.center = mean(bboxMap);
            objParam.speed = 0;
            objParam.objCloud= objCloud;
            objParam.bbox2d = get2dBoundingBox(bbCloud_img);
            objParam.label = objLabels{lidx};
            if strcmp(objParam.label,'traffic')
                objParam.label = 'traffic light';
            end
            
            objParam.trkCloud = [];
            objParam.centerTraj = [];
            if (strcmp(objParam.label, 'car') || strcmp(objParam.label, 'person')) && cIdx == 1
                objParam.state = 1;
                [velo3dRegMap, velo3dRegFoV_Color]= removeMovingObjects(velo3dRegMap(1:3,:)', velo3dRegFoV_Color, objParam.bboxMap);
            else
                objParam.state = 0;
            end
            objParam.frameNb = cIdx;
            objParam.ObjNb = length(objectList)+1;
            objectList{end+1} = objParam;
        else
            continue;
        end
        colorDraw = getDrawColor(colors, objLabels{lidx});
        figure(1000), hold on; plotCube(bboxMap, colorDraw); hold off;
        figure(1001); hold on;
        %                 plot(bbCloud_img(:,1),bbCloud_img(:,2),'s','Color', colorDraw);
        bbox2d = get2dBoundingBox(bbCloud_img);
        k = boundary(bbCloud_img(:,1),bbCloud_img(:,2));
        %         plot(bbCloud_img(k,1),bbCloud_img(k,2), '.-','Color', colorDraw, 'LineWidth', 3);
        rectangle('Position', bbox2d,...
            'EdgeColor', colorDraw, 'LineWidth', 3);
        set(gcf,'DefaultTextColor','red');
        if objParam.state == 0
            str = sprintf('ID:%02d %s', objParam.ObjNb,objParam.label);
            H= text(bbox2d(1)-20, bbox2d(2)-20, str);
            H.BackgroundColor = [0 0 0];
            H.FontSize = 15;
        end
        
        %         annotation('textbox',[0.5,0.4,0.1,0.1],'String',str)
        drawnow; hold off;
    end
    
    
    %% Estimate object velocity
    for objIdx = 1:length(objectList)
        if objectList{objIdx}.state ~= 1
            continue;
        end
        objectTmp = objectList{objIdx};
        noGndRegistT0_ = tform.T'*pinv(Tmapping)*[noGndRegistT0'; ones(1, size(noGndRegistT0, 1))];
        noGndRegistT0_ = noGndRegistT0_(1:3, :)';
        if ~isempty(objectTmp.trkCloud)
            [tOptimal, angularVelocity, linearVelocity, trkCloud, bboxMapUpdate, bbAxisUpdate] = ...
                locateObjectMotion(noGndRegistT0, noGndRegistT1, noGndRegistT0_, ...
                objectTmp.bboxMap, objectTmp.bbAxis, objectTmp.label,  objectTmp.trkCloud);
        else
            [tOptimal, angularVelocity, linearVelocity, trkCloud, bboxMapUpdate, bbAxisUpdate] = locateObjectMotion(noGndRegistT0,...
                noGndRegistT1, noGndRegistT0_, objectTmp.bboxCam, objectTmp.bbAxis, objectTmp.label);
        end
        %         if ~isempty(tOptimal)
        %             bboxMapUpdate = tOptimal*[objectTmp.bboxMap'; ones(1, 8)];
        %             bboxMapUpdate = bboxMapUpdate(1:3,:)';
        objectTmp.trkCloud = trkCloud;
        %         end
        if(norm(objectTmp.bboxMap-bboxMapUpdate)<0.5) || isempty(tOptimal)
            objectList{objIdx}.state = 0;
            figure(1001); hold on;
            str = sprintf('ID:%02d %s',objectList{objIdx}.ObjNb,...
                objectList{objIdx}.label);
            H= text(objectList{objIdx}.bbox2d(1) -20,...
                objectList{objIdx}.bbox2d(2) - 20, str);
            H.BackgroundColor = [0 0 0];
            H.FontSize = 15; hold off;
            
        else
            % draw object information
            if cIdx == 1
                figure(1001); hold on;
                str = sprintf('ID:%02d %s\nSpeed: %.01f m/s',objectList{objIdx}.ObjNb,...
                    objectList{objIdx}.label, linearVelocity);
                H= text(objectList{objIdx}.bbox2d(1)+objectList{objIdx}.bbox2d(3)+10,...
                    objectList{objIdx}.bbox2d(2)+0.5*objectList{objIdx}.bbox2d(4), str);
                H.BackgroundColor = [0 0 0];
                H.FontSize = 15; hold off;
                H.Color = [0 1 0];
            end
            [velo3dRegMap, velo3dRegFoV_Color]= removeMovingObjects(velo3dRegMap(1:3,:)', velo3dRegFoV_Color, objectTmp.bboxMap);
            centerTraj = [mean(objectTmp.bboxMap); mean(bboxMapUpdate)];
            objectTmp.bboxMap = bboxMapUpdate;
            objectTmp.bbAxis = bbAxisUpdate;
            colorDraw = getDrawColor(colors, objectTmp.label);
            objectTmp.centerTraj = [objectTmp.centerTraj; centerTraj];
            objectTmp.speed = linearVelocity;
            objectList{objIdx} = objectTmp;
%             figure(1000), hold on; plot3(centerTraj(:,1),centerTraj(:,2),centerTraj(:,3),'Color', colorDraw, 'LineWidth', 3);
%                         plotCube(objectTmp.bboxMap, colorDraw);
            hold off;
            if cIdx == params.frmEnd
%                 figure(1000), hold on; plotCube(objectTmp.bboxMap, colorDraw);
            end
        end
    end
    %     if strcmp(objLabels{lidx}, 'car') | strcmp(objLabels{lidx}, 'person')
    
    %     end
    fullMap = [fullMap;velo3dRegMap(1:3,:)'];
    fullMapColor = [fullMapColor; uint8(velo3dRegFoV_Color)];
    
    saveDir = './label3d/';
    if ~exist(saveDir)
        mkdir(saveDir);
    end
    if cIdx>0
        figure(1001); hold on; drawnow;
        pcshow(velo3dRegMap(1:3,:)', uint8(velo3dRegFoV_Color));
                drawnow; view([-1,1,1])
        hold off;
%         saveas(gcf, sprintf('%s%06d_2dlabels.png',saveDir,cIdx+shiftIdx));
    end
    bbox(:, 3) = bbox(:, 3) - bbox(:, 1);
    bbox(:, 4) = bbox(:, 4) - bbox(:, 2);
    %     for lidx = 1:size(bbox,1)
    %         figure(1001); hold on;
    %         rectangle('Position', bbox(lidx,:),...
    %             'EdgeColor','r', 'LineWidth', 0.5);
    %     end
    
end
figure(1000); hold on;
pcshow(fullMap, uint8(fullMapColor));
drawnow; view([-1,1,1])
hold off;
%% Reproduce Results
if 0
    load('label2d3dTransfer_ver1.mat');
    figure(4000), showPointCloud(fullMap, uint8(fullMapColor));
    for i = 1:length(objectList)
        figure(4000),hold on;
        objectTmp = objectList{i};
        colorDraw = getDrawColor(colors, objectTmp.label);
        if objectTmp.state
            centerTraj = objectTmp.centerTraj;
            plot3(centerTraj(:,1),centerTraj(:,2),centerTraj(:,3),'Color', colorDraw, 'LineWidth', 3);
        end
        plotCube(objectTmp.bboxMap, colorDraw);
    end
end
