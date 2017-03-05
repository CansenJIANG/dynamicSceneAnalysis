% function to plot the point cloud trajactory of size(3FxP)
function []=plotTrajectory(cube, color)
if nargin<2
    color = '-b';
end
plot3(cube(:,1),cube(:,2),cube(:,3), color); 
end