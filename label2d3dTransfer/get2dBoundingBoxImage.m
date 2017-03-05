%% function return the 2d bounding box of a set of image points
function [bb2d] = get2dBoundingBoxImage(imgProj)
if size(imgProj, 2) ~=2
    imgProj = imgProj';
end
bb2d = [min(imgProj(:,1)), min(imgProj(:,2)),...
    max(imgProj(:,1)) - min(imgProj(:,1)), max(imgProj(:,2)) - min(imgProj(:,2))];
end