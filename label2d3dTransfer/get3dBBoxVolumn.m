%% Function returns the 3D bounding volume and their edge length
function [bboxVol, bbX, bbY, bbZ] = get3dBBoxVolumn(bbox, bbAxis)
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

if nargin<2 
    bbAxis(:, 1) = (bbox(3,:) - bbox(1,:))/norm(bbox(3,:) - bbox(1,:));
    bbAxis(:, 2) = (bbox(3,:) - bbox(2,:))/norm(bbox(3,:) - bbox(2,:));
    bbAxis(:, 3) = (bbox(1,:) - bbox(4,:))/norm(bbox(1,:) - bbox(4,:));
end
bbox = bbox*bbAxis;
%% box volumn = x*y*z;
bbY = norm(bbox(1,:) - bbox(2,:));
bbX = norm(bbox(1,:) - bbox(3,:));
bbZ = norm(bbox(1,:) - bbox(4,:));
bboxVol = bbX*bbY*bbZ;
end