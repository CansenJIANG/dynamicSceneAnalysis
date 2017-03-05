% function to plot the 3d point trajectory
% cube: size Nx3
% color: choose plotting color, default blue
% function []=plot3dTrajectory(cube, color)
function []=plot2dTrajectory(cube, color)
if nargin<2
    color = '-b';
end
plot(cube(:,1),cube(:,2), color); 
end