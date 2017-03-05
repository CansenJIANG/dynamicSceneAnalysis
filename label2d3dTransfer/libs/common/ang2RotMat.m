% function to convert rotation angles around x, y, z axis to 3x3 rotation
% matrix, angles are in degrees.
% function R = ang2RotMat(x, y, z)

% Input
% x: rotation angle around X axis in degrees
% y: rotation angle around Y axis in degrees
% z: rotation angle around Z axis in degrees

% Output
% R: 3x3 rotation matrix in Rz*Ry*Rx seqence
function R = ang2RotMat(x, y, z)
if nargin <3
    y = x(2);
    z = x(3);
    x = x(1);
end

% convert from degree to radians
x = x*pi/180;
y = y*pi/180;
z = z*pi/180;

X = eye(3,3);
Y = eye(3,3);
Z = eye(3,3);

% Rx = [1,    0,      0]
%      [0,  cosx, -sinx]
%      [0,  sinx,  cosx]
X(2,2) = cos(x);
X(2,3) = -sin(x);
X(3,2) = sin(x);
X(3,3) = cos(x);


% RY = [cosy,    0,  siny]
%      [0,       1,     0]
%      [-siny,   0,  cosy]
Y(1,1) = cos(y);
Y(1,3) = sin(y);
Y(3,1) = -sin(y);
Y(3,3) = cos(y);


% Rz = [cosz, -sinz,    0]
%      [sinz,  cosz,    0]
%      [0,      0,      1]
Z(1,1) = cos(z);
Z(1,2) = -sin(z);
Z(2,1) = sin(z);
Z(2,2) = cos(z);

R = Z*Y*X;
end
