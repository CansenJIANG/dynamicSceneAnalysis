%% This function normalize the 3d data
%% [normalMat, pts3dTmp] = normalize3D(pts3d, nrmlMat)
%% Input: 3d Point, normalMat (not necessary)
%% Output: Normalized 3d Point, Normalization Matrix (if not provided)
function [normalMat, pts3dNrml] = normalize3D(pts3d, normalMat)
flatT = 0;
if size(pts3d, 1) ~= 3
    pts3d = pts3d';
    flatT = 1;
end
if nargin>1
    pts3dTmp = pts3d;
    pts3dTmp(4, :) = 1;
    pts3dTmp = normalMat*pts3dTmp;
    pts3dTmp = pts3dTmp(1:3, :);
    if flatT
        pts3dTmp = pts3dTmp';
    end
else
    lnth = length(pts3d);
    cen_x = sum(pts3d(1, :))/lnth;
    cen_y = sum(pts3d(2, :))/lnth;
    cen_z = sum(pts3d(3, :))/lnth;
    
    % translation
    normalTr = eye(4, 4);
    normalTr(1:3, end) = [-cen_x; -cen_y; -cen_z];
    
    % translate point cloud to center
    pts3dNrml = pts3d; pts3dNrml(4, :) = 1;
    pts3dNrml = normalTr*pts3dNrml;
    
    % get normalization scale
    distScl = sum(sqrt(sum(pts3dNrml(1:3, :).*pts3dNrml(1:3, :))))/lnth;
    distScl = sqrt(3)/distScl;
    
    normalRo = eye(4, 4);
    normalRo(1,1) = distScl;
    normalRo(2,2) = distScl;
    normalRo(3,3) = distScl;
    pts3dNrml = normalRo*pts3dNrml;
    if flatT
        pts3dNrml = pts3dNrml';
    end
    % final normalization matrix
    normalMat = normalRo*normalTr;
end
end