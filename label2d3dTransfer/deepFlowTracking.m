function [] = deepFlowTracking(imgDir, binDir, noGndDir, registDir, ...
    frmIdx, frmShift, P_velo_to_img, seqName, Tmapping)
deepMatchingExe = '/home/jiang/CvTools/deepmatching_1.2.2_c++/deepmatching ';
deepFlowExe     = '/home/jiang/CvTools/DeepFlow_release2.0/deepflow2 ';
noGndFiles = dir([noGndDir, '*_NoGnd.pcd']);

%% Starting Frame to track
% load velodyne points
fid  = fopen(sprintf('%s%06d.bin',binDir, frmIdx+frmShift),'rb');
veloT1 = fread(fid,[4 inf],'single')';
fclose(fid);
noGndT1 = loadpcd(sprintf('%s%s', noGndDir, noGndFiles(frmIdx).name))';
imgNameT1 = sprintf('%s%06d.png',imgDir, frmIdx+frmShift);
I1 = imread(imgNameT1);
idxBackT1 = veloT1(:,1)<0; veloT1(idxBackT1,:) = []; % remove all points behind image plane
noGndT1(idxBackT1,:) = [];

fid  = fopen(sprintf('%s%06d.bin',binDir, frmIdx+frmShift-1),'rb');
veloT0 = fread(fid,[4 inf],'single')';
fclose(fid);
noGndT0 = loadpcd(sprintf('%s%s', noGndDir, noGndFiles(frmIdx-1).name))';
imgNameT0 = sprintf('%s%06d.png',imgDir, frmIdx+frmShift-1);
I0 = imread(imgNameT0);
idxBackT0 = veloT0(:,1)<0; veloT0(idxBackT0,:) = []; % remove all points behind image plane
noGndT0(idxBackT0,:) = [];

%% Estimate Transformation from Map to Camera coordinate
% % transform T1
% cRegistT1 = loadpcd(sprintf('%s%s', registDir, noGndFiles(frmIdx).name))';
% cRegistT1 = pinv(Tmapping)*[cRegistT1; ones(1, size(cRegistT1,2))];
% noGndT1 = pinv(Tmapping)*[noGndT1; ones(1, size(noGndT1,2))];
% [tformT1, ~] = pcregrigid(pointCloud(cRegistT1(1:3,:)'),...
%     pointCloud(veloT1(:, 1:3)), 'Tolerance', [0.000001, 0.0000009], 'InlierRatio',1.0);
% noGndT1 = (tformT1.T')*noGndT1;
% noGndT1 = noGndT1(1:3, :)';
% 
% % transform T1
% cRegistT0 = loadpcd(sprintf('%s%s', registDir, noGndFiles(frmIdx-1).name))';
% cRegistT0 = pinv(Tmapping)*[cRegistT0; ones(1, size(cRegistT0,2))];
% noGndT0 = pinv(Tmapping)*[noGndT0; ones(1, size(noGndT0,2))];
% [tformT0, ~] = pcregrigid(pointCloud(cRegistT0(1:3,:)'),...
%     pointCloud(veloT0(:, 1:3)), 'Tolerance', [0.000001, 0.0000009], 'InlierRatio',1.0);
% noGndT0 = (tformT0.T')*noGndT0;
% noGndT0 = noGndT0(1:3, :)';

%% Project PointCloud to Image
% project to image plane (exclude luminance)
[veloImgT1, veloDepthT1] = project(veloT1(:,1:3),P_velo_to_img); veloImgT1 = veloImgT1';
[veloImgT0, veloDepthT0] = project(veloT0(:,1:3),P_velo_to_img); veloImgT0 = veloImgT0';

% Lets validate only those 3D and projected points which are within image
% Image size

Ix = size(I1, 2); Iy = size(I1, 1);
ptListT1 = veloImgT1(1,:)>0.5 & veloImgT1(2,:)>0.5 &...
    veloImgT1(1,:)< (Ix- 0.5) & veloImgT1(2,:)<(Iy- 0.5);
ptListT0 = veloImgT0(1,:)>0.5 & veloImgT0(2,:)>0.5 &...
    veloImgT0(1,:)< (Ix- 0.5) & veloImgT0(2,:)<(Iy- 0.5);

pts3DT1   = veloT1(ptListT1,1:3)'; veloDepthT1 = veloDepthT1(ptListT1);
noGndT1 = noGndT1(ptListT1, :);
proj3DT1  = veloImgT1(:,ptListT1);
proj3DT1(1,proj3DT1(1, :)<1) = 1;
proj3DT1(2,proj3DT1(2, :)<1) = 1;

pts3DT0   = veloT0(ptListT0,1:3)'; veloDepthT0 = veloDepthT0(ptListT0);
noGndT0 = noGndT1(ptListT0, :);
proj3DT0  = veloImgT0(:,ptListT0);
proj3DT0(1,proj3DT0(1, :)<1) = 1;
proj3DT0(2,proj3DT0(2, :)<1) = 1;


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
trkFeat = proj3DT0'; 
trkOF = getFeatureOFvelocity(trkFeat, vx, vy);

if 1|showFig
    %             figure(1), imshow(im1);hold on;
    %             plot(proj3D(1,:), proj3D(2,:),'.r'); % original projected points
    %             plot(trkFeat(:,1), trkFeat(:,2),'.g'); % with optical flow
    
    figure(2), imshow(im2); hold on;
    plot(trkFeat(:,1),trkFeat(:,2), '.','color',[0, 1, 0]);
    display('');
    %             plot(proj3Dtrk(1,:),proj3Dtrk(2,:), '.b');
end


%% Find tracked points from 2D projection of next 3d cloud
[knnIdx, knnDist] = knnsearch(proj3DT1', trkFeat);
%         lostIdx = unique([lostTrkIdx; outFoV]);
projOutlierIdx = find(knnDist>1.0);
lostIdx = unique( projOutlierIdx);
pts3Dcorr = proj3DT1(:, knnIdx); pts3Dcorr(:, lostIdx) = 0;
% traj3D{length(traj3D)+1} = pts3Dcorr;
ptsProj3Dcorr = proj3DT1(:, knnIdx); ptsProj3Dcorr(:, lostIdx) = 1.0;
% traj3Dproj{length(traj3Dproj)+1} = ptsProj3Dcorr;
% trajOF{length(trajOF)+1} = trkOF;


if 1 | showFig %| ( abs(idxframe-staSeq)==paramDF.nFrm)
    figure(300), imshow(im2); hold on;
    plot(proj3DT1(1,:),proj3DT1(2,:), '.','color',[0, 1, 0]);
    %             plot(proj3D(1,retrkIdx),proj3D(2,retrkIdx), 'sr');
end


end