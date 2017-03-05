% Synthetic cube data generation
% function cube = createCube(npts,scl,tr,ro)
% nPts: n points on the surface of the cube
% scl: size of the cube
% tr: translation from coordinate original
% ro: rotation around x, y, z axis
function cube = createCube(npts,scl,tr,ro, noCorner)
if nargin < 5
    noCorner = 0;
end

A = [0 0 0];
B = [1 0 0];
C = [0 1 0];
D = [0 0 1];
E = [0 1 1];
F = [1 0 1];
G = [1 1 0];
H = [1 1 1];
if noCorner
    P = [];
else
    P = [A;B;C;D;E;F;G;H];
end

% Generate random point on 3 surface
RandP1 = rand(npts,2); RandP1 = [RandP1, ones(npts,1)];
RandP2 = rand(npts,2); RandP2 = [RandP2(:,1), ones(npts,1), RandP2(:,2)];
RandP3 = rand(npts,2); RandP3 = [ ones(npts,1),RandP3(:,1), RandP3(:,2)];

% Initialize the position of different objects
cube = scl*[P; RandP1;RandP2;RandP3];

% Get rotation matrix from rotation angles
cube = rigidTransform(cube, ro,tr);
end