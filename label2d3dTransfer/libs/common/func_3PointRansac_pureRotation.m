%% Function to solve rigid transformation using 3-point RANSAC algorithm.
%% This function can solve pure rotation as well as rotation + translation.
%% The default setting consider the transformation contains rotation and 
%% translation.
%% If you want to estimate pure rotation, remove the translation terms.
%% C. Jiang
function transMat = func_3PointRansac_pureRotation(corrRef, corrNew)
if nargin <2
    corrRef = 10*rand([3, 3]);
    theta_x = 5*pi/180; theta_y = 10*pi/180; theta_z=10*pi/180;
    Rx = [1,0,0;0,cos(theta_x),sin(theta_x);0,-sin(theta_x),cos(theta_x)]; % rotation around x axis
    Ry = [cos(theta_y),0,-sin(theta_y);0,1,0;sin(theta_y),0,cos(theta_y)]; % rotation around y axis
    Rz = [cos(theta_z),sin(theta_z),0;-sin(theta_z),cos(theta_z),0;0,0,1]; % rotation around z axis
    Tr_xyz = [0; 0; 0];
    GT = [Rz*Ry*Rx, Tr_xyz; 0, 0, 0, 1];
    corrNew = GT* [corrRef; ones(1, 3)];
    corrNew(1, :) = corrNew(1, :)./ corrNew(end, :);
    corrNew(2, :) = corrNew(2, :)./ corrNew(end, :);
    corrNew(3, :) = corrNew(3, :)./ corrNew(end, :);
    corrNew(4, :) = corrNew(4, :)./ corrNew(end, :);
end
% build linear system Ax = b;
rows = 3*3;
cols = 6;
A = zeros(rows, cols);
b = zeros(rows, 1);
for i=1:3
    X1 = corrRef(1, i);
    Y1 = corrRef(2, i);
    Z1 = corrRef(3, i);
    X0 = corrNew(1, i);
    Y0 = corrNew(2, i);
    Z0 = corrNew(3, i);
    A(3*i-2, 1) = 0;      A(3*i -2, 2) = Z0+Z1;  A(3*i-2, 3) = -Y0-Y1; A(3*i-2, 4) = 1;
    A(3*i-1, 1) = -Z0-Z1; A(3*i-1,  2) = 0;      A(3*i-1, 3) = X0+X1;  A(3*i-1, 5) = 1;
    A(3*i  , 1) = Y0+Y1;  A(3*i,    2) = -X0-X1; A(3*i  , 3) = 0;      A(3*i  , 6) = 1;
    
    b(3*i-2) = X1-X0;
    b(3*i-1) = Y1-Y0;
    b(3*i  ) = Z1-Z0;
end

% Solve A*x = b; solveX = [Rx, Ry, Rz, T'x, T'y, T'z];
solveX = A\b;

%% recover translation
I_Vx = [    1,       solveX(3),    -solveX(2);
    -solveX(3),      1,         solveX(1);
    solveX(2),  -solveX(1),        1   ];

Tx   = [ solveX(4);  solveX(5);  solveX(6)];
Tr = inv(I_Vx)*Tx;

%% recover rotation
IplusVx = [    1,       -solveX(3),    solveX(2);
    solveX(3),      1,        -solveX(1);
    -solveX(2),  solveX(1),         1   ];

Ro = inv(I_Vx)*IplusVx;

transMat = [Ro, Tr; 0, 0, 0, 1];

%% Check estimation error
if nargin < 2
    corrNew_ = transMat*corrNew;
    corrNew_(1, :) = corrNew_(1, :)./ corrNew_(end, :);
    corrNew_(2, :) = corrNew_(2, :)./ corrNew_(end, :);
    corrNew_(3, :) = corrNew_(3, :)./ corrNew_(end, :);
    corrNew_(4, :) = corrNew_(4, :)./ corrNew_(end, :);
    err = 0;
    for i = 1:3
        err = err + norm(corrNew_(1:3, i) - corrRef(1:3, i));
    end
    err = err/3;
    display( ['Estimation Error is: ', num2str(err)]);
    display( 'Ground Truth Transformation Matrix: ');
    display(GT);
    display( 'Estimated Transformation Matrix: ');
    display(inv(transMat));
end
end