% function to plot the 3d point trajectory
% cube: size Nx3
% color: choose plotting color, default blue
% function []=plot3dTrajectory(cube, color)
function []=plot3dTrajectory(cube, color)
if nargin<2
    color = '-*b';
end
% remove zeros padding
cube(find(sum(cube,2) == 0),:) = [];

plot3(cube(:,1),cube(:,2),cube(:,3), color, 'markers', 12); 
end