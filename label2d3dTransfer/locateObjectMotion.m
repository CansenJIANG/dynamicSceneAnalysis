function [tOptimal, angularVelocity, linearVelocity, inlierCloud, bboxMap, bbAxis] =...
    locateObjectMotion(noGndRegistT0, noGndRegistT1, ...
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
    
if nargin>6
    bbAxis(:, 1) =  (bboxMap(3,:)'-bboxMap(1,:)')/norm((bboxMap(3,:)'-bboxMap(1,:)'));
    bbAxis(:, 2) =  (bboxMap(2,:)'-bboxMap(1,:)')/norm((bboxMap(2,:)'-bboxMap(1,:)'));
    bbAxis(:, 3) =  (bboxMap(4,:)'-bboxMap(1,:)')/norm((bboxMap(4,:)'-bboxMap(1,:)'));
    for i = 1:3
        v = bbAxis(:, i);
        bboxMax(i) = max(bboxMap*v)+0.25;
        bboxMin(i) = min(bboxMap*v)-0.25;
        projAxis(:, i) = noGndRegistT0*v;
    end
    
    bbIdx_x = intersect(find(projAxis(:,1)<bboxMax(1)), ...
        find(projAxis(:,1)>bboxMin(1)));
    bbIdx_y = intersect(find(projAxis(:,2)<bboxMax(2)), ...
        find(projAxis(:,2)>bboxMin(2)));
    bbIdx_z = intersect(find(projAxis(:,3)<bboxMax(3)), ...
        find(projAxis(:,3)>bboxMin(3)));
    bbIdx = intersect(bbIdx_x, bbIdx_y);
    bbIdx = intersect(bbIdx, bbIdx_z);
else
    for i = 1:3
        v = bbAxis(:, i);
        bboxMax(i) = max(bboxMap*v);
        bboxMin(i) = min(bboxMap*v);
        projAxis(:, i) = cloudFoV*v;
    end
    
    bbIdx_x = intersect(find(projAxis(:,1)<bboxMax(1)), ...
        find(projAxis(:,1)>bboxMin(1)));
    bbIdx_y = intersect(find(projAxis(:,2)<bboxMax(2)), ...
        find(projAxis(:,2)>bboxMin(2)));
    bbIdx_z = intersect(find(projAxis(:,3)<bboxMax(3)), ...
        find(projAxis(:,3)>bboxMin(3)));
    bbIdx = intersect(bbIdx_x, bbIdx_y);
    bbIdx = intersect(bbIdx, bbIdx_z);
end
% objCloud = cloudFoV(bbIdx, :);
% [bbCloud_img, bbCloud_depth] = project(objCloud, P_velo_to_img);

bbCloudT0 = noGndRegistT0(bbIdx, :);
bbCloudT1 = noGndRegistT1(bbIdx, :);
% figure, showPointCloud(bbCloudT0); hold on; showPointCloud(bbCloudT1, [1 0 0]);
if length(bbCloudT1) < 4
    angularVelocity = [];
    tOptimal = [];
    linearVelocity = 0;
    inlierCloud = [];
    return;
end
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
%% Adjust bbox center
% bbox2 = bbox + repmat(median(bbCloudT0, 1) - mean(bbox), [8 1]);
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
    boxVelocity = norm(mean(bboxMap) - mean(bboxT1(1:3,:)'))*10;
    bboxMap = bboxT1(1:3,:)';
else
    boxVelocity = norm(mean(bboxMap) - mean(bboxT1))*10;
    bboxMap = bboxT1;
    bbAxis = bbAxisT1;
end
bbCloudTrans = tOptimal* [bbCloudT0'; ones(1, length(bbCloudT0))];
[alpha, beta, gamma] = rotation3dToEulerAngles(tOptimal(1:3, 1:3));
angularVelocity = norm([alpha, beta, gamma])*pi/180*10;
if 1
    pcDist = (bbCloudT1(inlierIdxOpt,:) - bbCloudT0(inlierIdxOpt,:));
    motDist = sqrt(sum(pcDist.*pcDist,2));
    linearVelocity = mean(motDist)*10;
else
tOptimal(1:2, 3) = 0;
tOptimal(3, 1:2) = 0;
% tOptimal(3, end) = 0;
tOptimal(2, end) = tOptimal(2, end) + tOptimal(3, end);
linearVelocity = norm(tOptimal(1:2, end))*10;
if linearVelocity > 8
    tOptimal(2, end) = tOptimal(2, end) + tOptimal(3, end);
    linearVelocity = norm(tOptimal(1:2, end))*10;
end
end
% figure, showPointCloud(bbCloudT1, [1 0 0]); hold on;
% showPointCloud(bbCloudTrans(1:3, :)', [0 0 1]); hold off;

end