%% Segment point cloud within a 3d bounding box
%% [maxX, minX, maxY, minY, maxZ, minZ]
function cloudSeg = segmentCloudBbox3d(cloud, bbox3d, offsetXYZ)

if nargin < 3
    % [maxX, minX, maxY, minY, maxZ, minZ]
    offsetXYZ = zeros(6, 1);
end
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
bbAxis(:, 1) =  (bbox3d(3,:)'-bbox3d(1,:)')/norm((bbox3d(3,:)'-bbox3d(1,:)'));
bbAxis(:, 2) =  (bbox3d(2,:)'-bbox3d(1,:)')/norm((bbox3d(2,:)'-bbox3d(1,:)'));
bbAxis(:, 3) =  (bbox3d(1,:)'-bbox3d(4,:)')/norm((bbox3d(4,:)'-bbox3d(1,:)'));
for i = 1:3
    v = bbAxis(:, i);
    bboxMax(i) = max(bbox3d*v);
    bboxMin(i) = min(bbox3d*v);
    projAxis(:, i) = cloud*v;
end
% Compute point cloud inside bounding box
bbIdx_x = intersect(find(projAxis(:,1)<bboxMax(1) + offsetXYZ(1) ), ...
    find(projAxis(:,1)>bboxMin(1) + offsetXYZ(2) ));
bbIdx_y = intersect(find(projAxis(:,2)<bboxMax(2) + offsetXYZ(3)), ...
    find(projAxis(:,2)>bboxMin(2) + offsetXYZ(4)));
bbIdx_z = intersect(find(projAxis(:,3)<bboxMax(3)+ offsetXYZ(5)), ...
    find(projAxis(:,3)>bboxMin(3) + offsetXYZ(6)));
bbIdx = intersect(bbIdx_x, bbIdx_y);
bbIdx = intersect(bbIdx, bbIdx_z);

cloudSeg = cloud(bbIdx, :);
if 0
    figure(20), showPointCloud(cloud); hold on; 
     showPointCloud(cloudSeg, [1 0 0]);
end
end