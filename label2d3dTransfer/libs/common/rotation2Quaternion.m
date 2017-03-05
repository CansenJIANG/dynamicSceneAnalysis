% function to convert 3x3 rotation matrix or euler angles to 4x1 quaternion
% rotation representation: q = cos(theta/2)+(Ux*i + Uy*j + Uz*k)sin(theta/2)
% function adapted from:
% http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche52.html

% Input:
% R: 3x3 rotation matrix
% R: if R is 3 euler angles around X, Y, Z axis, the default rotation
% sequence is Rz*Ry*Rx.

% Output:
% Q: 1x4 quaternion representation of rotation
% u: 1x3 rotation axis
% theta: rotation angle

function [Q, u, theta] = rotation2Quaternion(R)
if(size(R,1)*size(R,2)==3)
    R = ang2RotMat(R(1), R(2), R(3));
end
    
r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);

q0 = ( r11 + r22 + r33 + 1.0) / 4.0;
q1 = ( r11 - r22 - r33 + 1.0) / 4.0;
q2 = (-r11 + r22 - r33 + 1.0) / 4.0;
q3 = (-r11 - r22 + r33 + 1.0) / 4.0;
if q0 < 0.0
    q0 = 0.0;
end
if q1 < 0.0
    q1 = 0.0;
end
if q2 < 0.0
    q2 = 0.0;
end
if q3 < 0.0
    q3 = 0.0;
end

q0 = sqrt(q0);
q1 = sqrt(q1);
q2 = sqrt(q2);
q3 = sqrt(q3);

if(q0 >= q1 && q0 >= q2 && q0 >= q3)
    q0 = q0;
    q1 = q1*sign(r32 - r23);
    q2 = q2*sign(r13 - r31);
    q3 = q3*sign(r21 - r12);
elseif(q1 >= q0 && q1 >= q2 && q1 >= q3)
    q0 = q0*sign(r32 - r23);
    q1 = q1;
    q2 = q2*sign(r21 + r12);
    q3 = q3*sign(r13 + r31);
elseif(q2 >= q0 && q2 >= q1 && q2 >= q3)
    q0 = q0*sign(r13 - r31);
    q1 = q1*sign(r21 + r12);
    q2 = q2;
    q3 = q3*sign(r32 + r23);
elseif(q3 >= q0 && q3 >= q1 && q3 >= q2)
    q0 = q0*sign(r21 - r12);
    q1 = q1*sign(r31 + r13);
    q2 = q2*sign(r32 + r23);
    q3 = q3;
else
    display('coding error');
end

Q = [q0, q1, q2, q3];
Q = Q/norm(Q);

theta = 2*acos(q0);
ux = q1/sin(theta/2);
uy = q2/sin(theta/2);
uz = q3/sin(theta/2);
u = [ux, uy, uz];
theta = theta*180/pi;
end

