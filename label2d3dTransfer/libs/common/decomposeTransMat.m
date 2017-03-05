% function decompose the 4x4 transformation matrix in homogeneous
% coordinate to quaternion representation of rotation, and X, Y, Z
% translation.

% Input:
% T: 4x4 transformation matrix in homogeneous coordinate consist of a 3x3
% rotation matrix and a 3x1 translation matrix.
% The quaternion representation of rotation has not constrain w.r.t. 
% different type of rotation sequence in euler angles.
% 3x3 rotation matrix combination types 'Rxyz', 'Rxzy', 'Ryxz',
% 'Ryzx', 'Rzxy', 'Rzyx', 'Rxoyx1', 'Rxozx1', 'Ryoxy1', 'Ryozy1',
% 'Rzoxz1', 'Rzoyz1'.

% Output:
% q:  1x4 quaternion representation of rotation
% tr: 1x3 translation along X, Y, Z axis


function [q, tr] = decomposeTransMat(T)
    % 3x3 rotation matrix
    R = T(1:3,1:3);
    q = rotation2Quaternion(R);
    
    % 3x1 translation matix
    tr = T(1:3,4)';
end