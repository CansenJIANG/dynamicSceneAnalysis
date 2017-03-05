function [cloud, cloudColor] = removeMovingObjects(cloud, cloudColor, bbox, bbAxis)

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
if nargin<4
    bbAxis(:, 1) =  (bbox(3,:)'-bbox(1,:)')/norm((bbox(3,:)'-bbox(1,:)'));
    bbAxis(:, 2) =  (bbox(2,:)'-bbox(1,:)')/norm((bbox(2,:)'-bbox(1,:)'));
    bbAxis(:, 3) =  (bbox(4,:)'-bbox(1,:)')/norm((bbox(4,:)'-bbox(1,:)'));
end
    for i = 1:3
        v = bbAxis(:, i);
        bboxMax(i) = max(bbox*v) + 0.75;
        bboxMin(i) = min(bbox*v) - 0.75;
        projAxis(:, i) = cloud*v;
    end
    
    bbIdx_x = intersect(find(projAxis(:,1)<bboxMax(1)), ...
        find(projAxis(:,1)>bboxMin(1)));
    bbIdx_y = intersect(find(projAxis(:,2)<bboxMax(2)), ...
        find(projAxis(:,2)>bboxMin(2)));
    bbIdx_z = intersect(find(projAxis(:,3)<bboxMax(3)), ...
        find(projAxis(:,3)>bboxMin(3)));
    bbIdx = intersect(bbIdx_x, bbIdx_y);
    bbIdx = intersect(bbIdx, bbIdx_z);

    cloud(bbIdx, :) = []; cloud = cloud';
    cloudColor(bbIdx, :) = [];
end
