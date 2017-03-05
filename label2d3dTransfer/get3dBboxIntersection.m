function [volInter3d] = get3dBboxIntersection(bbox3d1, bbox3d2)
%
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

bbox1 = [bbox3d1(1, 1:2); bbox3d1(3, 1:2); bbox3d1(7, 1:2);bbox3d1(2, 1:2)];
bbox2 = [bbox3d2(1, 1:2); bbox3d2(3, 1:2); bbox3d2(7, 1:2);bbox3d2(2, 1:2)];
inter2d = rectIntersect(bbox1, bbox2);
volInter3d = 0;
if inter2d~=0
    areaInter = areaIntersection2dPolygon(bbox1, bbox2, 100);
    minZ = max([min(bbox3d1(:,3)), min(bbox3d2(:,3))]);
    maxZ = min([max(bbox3d1(:,3)), max(bbox3d2(:,3))]);
    volInter3d = areaInter*(maxZ-minZ);
end
end