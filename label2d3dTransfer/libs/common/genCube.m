% Synthetic data generation
function cube = genCube()
close all;
figure, 
A = [0 0 0];
B = [1 0 0];
C = [0 1 0];
D = [0 0 1];
E = [0 1 1];
F = [1 0 1];
G = [1 1 0];
H = [1 1 1];
P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
plot3(P(:,1),P(:,2),P(:,3)); hold on;
axis([-5 5 -5 5 -5 5]); 
xlabel('X'); ylabel('Y'); zlabel('Z')

npts = 10;
RandP1 = rand(npts,2); RandP1 = [RandP1, ones(npts,1)];
RandP2 = rand(npts,2); RandP2 = [RandP2(:,1), ones(npts,1), RandP2(:,2)];
RandP3 = rand(npts,2); RandP3 = [ ones(npts,1),RandP3(:,1), RandP3(:,2)];
plot3(RandP1(:,1),RandP1(:,2),RandP1(:,3),'r*');
plot3(RandP2(:,1),RandP2(:,2),RandP2(:,3),'g*');
plot3(RandP3(:,1),RandP3(:,2),RandP3(:,3),'b*');



% Initialize the position of different objects
S0 = [P; RandP1;RandP2;RandP3];

S1 = [P; RandP1;RandP2;RandP3] + 3*ones(3*npts+length(P),3);
plot3(S1(1:length(P),1),S1(1:length(P),2),S1(1:length(P),3));
plot3(S1(:,1),S1(:,2),S1(:,3), 'r*');

S2 = [P; RandP1;RandP2;RandP3] - 2*ones(3*npts+length(P),3);
plot3(S2(1:length(P),1),S2(1:length(P),2),S2(1:length(P),3));
plot3(S2(:,1),S2(:,2),S2(:,3), 'r*');

S3 = [P; RandP1;RandP2;RandP3] + [3*zeros(3*npts+length(P),1), 2*ones(3*npts+length(P),2)];
plot3(S3(1:length(P),1),S3(1:length(P),2),S3(1:length(P),3));
plot3(S3(:,1),S3(:,2),S3(:,3), 'r*');

hold off;

end