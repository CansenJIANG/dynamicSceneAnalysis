function [noGndProjT0, noGndRegistT0, noGndProjT1, noGndRegistT1] = ...
    deepFlowTracking2(imgDir, binDir, noGndDir, registDir, ...
    frmIdx, frmShift, P_velo_to_img, seqName, Tmapping)
deepMatchingExe = '/home/jiang/CvTools/deepmatching_1.2.2_c++/deepmatching ';
deepFlowExe     = '/home/jiang/CvTools/DeepFlow_release2.0/deepflow2 ';
noGndFiles = dir([noGndDir, '*_NoGnd.pcd']);

[pts3dT0, proj3dT0, noGndT0, noGndProjT0, noGndRegistT0, noGndDepthT0, I0, imgNameT0] = ...
    associate2d3dData(imgDir, binDir, registDir, noGndDir,...
    frmIdx, frmShift, P_velo_to_img, Tmapping);

[pts3dT1, proj3dT1, noGndT1, noGndProjT1, noGndRegistT1, noGndDepthT1, I1, imgNameT1] = ...
    associate2d3dData(imgDir, binDir, registDir, noGndDir,...
    frmIdx+1, frmShift, P_velo_to_img, Tmapping);
%% Compute Dense Optical Flow
floName = [sprintf('/home/jiang/CvTools/DeepFlow_release2.0/flo/%s_%06d_%06d.flo',seqName,frmIdx+frmShift-1, frmIdx+frmShift)];
im1 = im2single(I0);
im2 = im2single(I1);
deepFlowSet = [floName,' -match -kitti'];% -sintel , -middlebury
command = [deepMatchingExe, imgNameT0,' ',imgNameT1, ' | ', ...
    deepFlowExe, imgNameT0,' ', imgNameT1,' ',deepFlowSet];
if exist(floName)
    flo = readFlowFile(floName);
    vx = flo(:,:,1); vy = flo(:,:,2);
else
    tic;
    [status,cmdout] = system(command);
    flo = readFlowFile(floName);
    vx = flo(:,:,1); vy = flo(:,:,2);
    toc
end

%% Compute OF for 2D projections on img1
trkFeat = noGndProjT0'; 
[trkFeat, trkOF] = getFeatureOFvelocity(trkFeat, vx, vy);
showFig = 0;
if showFig
    %             figure(1), imshow(im1);hold on;
    %             plot(proj3D(1,:), proj3D(2,:),'.r'); % original projected points
    %             plot(trkFeat(:,1), trkFeat(:,2),'.g'); % with optical flow
    
    figure(2), clf; imshow(im2); hold on;
    plot(trkFeat(:,1),trkFeat(:,2), '.','color',[0, 1, 0]);
    plot(noGndProjT1(1, :),noGndProjT1(2, :), '.','color',[1, 0, 0]);
    display('');
    %             plot(proj3Dtrk(1,:),proj3Dtrk(2,:), '.b');
end


%% Find tracked points from 2D projection of next 3d cloud
[knnIdx, knnDist] = knnsearch(noGndProjT1', trkFeat);
%         lostIdx = unique([lostTrkIdx; outFoV]);
projOutlierIdx = find(knnDist>2.0);
depthDist = abs(sum(noGndRegistT0 - noGndRegistT1(knnIdx, :),2));
lostIdx = unique( [projOutlierIdx; find(depthDist>2.5)]);
trkFeat(lostIdx, :) = [];
noGndProjT0(:, lostIdx) = [];
noGndRegistT0(lostIdx, :) = [];
noGndProjT1 = noGndProjT1(:, knnIdx);
noGndProjT1(:, lostIdx) = [];
noGndRegistT1 = noGndRegistT1(knnIdx, :);
noGndRegistT1(lostIdx, :) = [];

if showFig
    figure(2), clf; imshow(im2); hold on;
    plot(trkFeat(:,1),trkFeat(:,2), '.','color',[0, 1, 0]);
    plot(noGndProjT1(1, :),noGndProjT1(2, :), '.','color',[1, 0, 0]);
    display('');
    %             plot(proj3Dtrk(1,:),proj3Dtrk(2,:), '.b');
    figure(3); clf; showPointCloud(noGndRegistT0, [1 0 0]); hold on;
    showPointCloud(noGndRegistT1, [0 0 1]); hold on;
    qv = [noGndRegistT1 - noGndRegistT0];
    quiver3(noGndRegistT0(:,1), noGndRegistT0(:,2), noGndRegistT0(:,3),...
        qv(:,1), qv(:,2), qv(:,3), 0);
end


end
