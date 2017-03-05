% function to plot the cube of size(Nx3)
% function []=plotCube(cube, color, color_)
% cube: Nx3 matrix
% color: color type for cube edges
% color_: color for points on the surface
% 
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

function []=plotCube(cube, color, color_)
if nargin<2
    color = '-g';
    color_ = '.g';
else
    color = ['-', color];
    color_ = ['.',color];
end

% get vertices from cube
A = cube(1,:);
B = cube(2,:);
C = cube(3,:);
D = cube(4,:);
E = cube(5,:);
F = cube(6,:);
G = cube(7,:);
H = cube(8,:);
Vertices = [A;B; F;H; G;C; A;D; E;H; F;D; E;C; G;B];

plot3(Vertices(:,1),Vertices(:,2),Vertices(:,3), color); 
plot3(cube(:,1),cube(:,2),cube(:,3), color_, 'MarkerSize', 10); 
end