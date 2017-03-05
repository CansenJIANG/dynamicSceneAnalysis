function [pts3D, proj3D, noGnd, noGndProj, noGndRegist, noGndDepth, I, imgName] =...
    associate2d3dData(imgDir, binDir, registDir, noGndDir, frmIdx, frmShift, P_velo_to_img, Tmapping)
%% Starting Frame to track
% load velodyne points
noGndFiles = dir([noGndDir, '*_NoGnd.pcd']);
fid  = fopen(sprintf('%s%06d.bin',binDir, frmIdx+frmShift),'rb');
veloT1 = fread(fid,[4 inf],'single')';
fclose(fid);
noGnd = loadpcd(sprintf('%s%s', noGndDir, noGndFiles(frmIdx).name));
noGndRegist = loadpcd(sprintf('%s%s', noGndDir, noGndFiles(frmIdx).name))';
imgName = sprintf('%s%06d.png',imgDir, frmIdx+frmShift);
I = imread(imgName);

%% Estimate Transformation from Map to Camera coordinate
% % transform T1
cRegistT1 = loadpcd(sprintf('%s%s.pcd', registDir, noGndFiles(frmIdx).name(1:end-10)));
cRegistT1 = pinv(Tmapping)*[cRegistT1; ones(1, size(cRegistT1,2))];
noGnd = pinv(Tmapping)*[noGnd; ones(1, size(noGnd,2))];
[tformT1, cRegist] = pcregrigid(pointCloud(cRegistT1(1:3,:)'),...
    pointCloud(veloT1(:, 1:3)), 'Tolerance', [0.000001, 0.0000009], 'InlierRatio',1.0);
noGnd = (tformT1.T')*noGnd;
noGnd = noGnd(1:3, :)';

idxBackT1 = find(veloT1(:,1)<0); veloT1(idxBackT1,:) = []; % remove all points behind image plane
noGndRmIdxT1 = [];
noGndRmIdxT1 = find(noGnd(:,1)<0); noGnd(noGndRmIdxT1, :) = [];
noGndRegist(noGndRmIdxT1, :) = [];


%% Project PointCloud to Image
% project to image plane (exclude luminance)
[veloImgT1, veloDepthT1] = project(veloT1(:,1:3),P_velo_to_img); veloImgT1 = veloImgT1';
[noGndProj, noGndDepth] = project(noGnd(:,1:3),P_velo_to_img); noGndProj = noGndProj';

% Lets validate only those 3D and projected points which are within image
% Image size
Ix = size(I, 2); Iy = size(I, 1);
ptListT1 = veloImgT1(1,:)>0.5 & veloImgT1(2,:)>0.5 &...
    veloImgT1(1,:)< (Ix- 0.5) & veloImgT1(2,:)<(Iy- 0.5);
ptListNoGnd = noGndProj(1,:)>0.5 & noGndProj(2,:)>0.5 &...
    noGndProj(1,:)< (Ix- 0.5) & noGndProj(2,:)<(Iy- 0.5);

pts3D = veloT1(ptListT1,1:3)'; veloDepthT1 = veloDepthT1(ptListT1);
proj3D  = veloImgT1(:,ptListT1);
proj3D(1,proj3D(1, :)<1) = 1;
proj3D(2,proj3D(2, :)<1) = 1;

noGnd = noGnd(ptListNoGnd, :); noGndDepth = noGndDepth(ptListNoGnd);
noGndProj = noGndProj(:, ptListNoGnd);
noGndProj(1,noGndProj(1, :)<1) = 1;
noGndProj(2,noGndProj(2, :)<1) = 1;
noGndRegist = noGndRegist(ptListNoGnd, :);
end