function [tOptimal, angularVelocity, linearVelocity, inlierCloud, bboxMap, bbAxis] =...
    estimateObjectMotion(noGndRegistT0, noGndRegistT1, ...
    cloudFoV, bboxMap, bbAxis, label, trkCloud)
warning off;
%   Front surface
% A *__________* B
%   |          |
%   |          |
%   |          |
%   |          |
% D *__________* F

%   Back surface
% C *__________* G
%   |          |
%   |          |
%   |          |
%   |          |
% E *__________* H

% Estimate bounding box direction
bbAxis(:, 1) =  (bboxMap(3,:)'-bboxMap(1,:)')/norm((bboxMap(3,:)'-bboxMap(1,:)'));
bbAxis(:, 2) =  (bboxMap(2,:)'-bboxMap(1,:)')/norm((bboxMap(2,:)'-bboxMap(1,:)'));
bbAxis(:, 3) =  (bboxMap(4,:)'-bboxMap(1,:)')/norm((bboxMap(4,:)'-bboxMap(1,:)'));
for i = 1:3
    v = bbAxis(:, i);
    bboxMax(i) = max(bboxMap*v)+0.25;
    bboxMin(i) = min(bboxMap*v)-0.25;
    projAxis(:, i) = noGndRegistT0*v;
end
% Compute point cloud inside bounding box
bbIdx_x = intersect(find(projAxis(:,1)<bboxMax(1)), ...
    find(projAxis(:,1)>bboxMin(1)));
bbIdx_y = intersect(find(projAxis(:,2)<bboxMax(2)), ...
    find(projAxis(:,2)>bboxMin(2)));
bbIdx_z = intersect(find(projAxis(:,3)<bboxMax(3)), ...
    find(projAxis(:,3)>bboxMin(3)));
bbIdx = intersect(bbIdx_x, bbIdx_y);
bbIdx = intersect(bbIdx, bbIdx_z);

bbCloudT0 = noGndRegistT0(bbIdx, :);
bbCloudT1 = noGndRegistT1(bbIdx, :);
if 0
     figure, showPointCloud(bbCloudT0, [0 0 1]); hold on; 
     showPointCloud(bbCloudT1, [1 0 0]);
end
if length(bbCloudT1) < 4
    angularVelocity = 0;
    tOptimal = [];
    linearVelocity = 0;
    inlierCloud = [];
    return;
end
%% 3-point RANSAC registration
minErr = 10000; maxInlierNb = -10; inlierDist = 0.08; tOptimal = [];
inlierIdxOpt = [];
for i= 1:20000
    r = randi([1 length(bbCloudT1)], 1,3);
    rSmpRef = bbCloudT1(r, :)';
    rSmpModel = bbCloudT0(r, :)';
    Tsac = func_3PointRansac(rSmpRef, rSmpModel);
    bbCloudTrans = Tsac* [bbCloudT0'; ones(1, length(bbCloudT0))];
    diff = (bbCloudTrans(1:3, :) - bbCloudT1');
    diff = sqrt(sum( diff.*diff));
    inlierIdx = find(diff<inlierDist);
    if maxInlierNb <= length(inlierIdx)
        diff = diff(inlierIdx);
        avgErr = sum(diff)/length(diff);
        if maxInlierNb == length(inlierIdx) &&  avgErr > minErr
            continue;
        end
        minErr = avgErr;
        tOptimal = Tsac;
        maxInlierNb = length(inlierIdx);
        inlierCloud = bbCloudT1(inlierIdx, :);
        inlierIdxOpt = inlierIdx;
    end
end

%% Estimate Object Speed
pcDist = (bbCloudT1(inlierIdxOpt,1:2) - bbCloudT0(inlierIdxOpt,1:2));
motDist = sqrt(sum(pcDist.*pcDist,2));
if length(motDist)>100
    linearVelocity = median(motDist)*10;
else
    linearVelocity = mean(motDist)*10;
end
if linearVelocity < 2.0
    angularVelocity = 0;
    tOptimal = [];
    linearVelocity = 0;
    return;
end

%% Adjust bbox center
[bboxT1, mu, bbAxisT1] = fitBBoxHorizontal(bbCloudT1);
boxScale = 2;
if strcmp(label,'car')
    boxScale = 2.5;
end
if get3dBBoxVolumn(bboxT1)/get3dBBoxVolumn(bboxMap) > boxScale ||...
        get3dBBoxVolumn(bboxT1)/get3dBBoxVolumn(bboxMap) < 0.5
    tOptimal(1:2, 3) = 0;
    tOptimal(3, 1:2) = 0;
    tOptimal(3, end) = 0;
    bboxT1 = tOptimal*[bboxMap'; ones(1, 8)];
%     boxVelocity = norm(mean(bboxMap) - mean(bboxT1(1:3,:)'))*10;
    bboxMap = bboxT1(1:3,:)';
else
%     boxVelocity = norm(mean(bboxMap) - mean(bboxT1))*10;
    bboxMap = bboxT1;
    bbAxis = bbAxisT1;
end
% bbCloudTrans = tOptimal* [bbCloudT0'; ones(1, length(bbCloudT0))];
[alpha, beta, gamma] = rotation3dToEulerAngles(tOptimal(1:3, 1:3));
angularVelocity = norm([alpha, beta, gamma])*pi/180*10;

% figure, showPointCloud(bbCloudT1, [1 0 0]); hold on;
% showPointCloud(bbCloudTrans(1:3, :)', [0 0 1]); hold off;

end