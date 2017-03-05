% Function for 3x3 rotation matrix decomposition. 
% Function includes all the possible ways in decomposing the 3x3 rotation
% matrices.

% INPUT
% ro: 3x3 rotation matrix
% type : 3x3 rotation matrix combination types 'Rxyz', 'Rxzy', 'Ryxz',
% 'Ryzx', 'Rzxy', 'Rzyx', 'Rxoyx1', 'Rxozx1', 'Ryoxy1', 'Ryozy1',
% 'Rzoxz1', 'Rzoyz1'.
% Rxyz : rotation matrix combined by Rx*Ry*Rz
% Rxzy : rotation matrix combined by Rx*Rz*Ry
% Ryxz : rotation matrix combined by Ry*Rx*Rz
% Ryzx : rotation matrix combined by Ry*Rz*Rx
% Rzxy : rotation matrix combined by Rz*Rx*Ry
% Rzyx : rotation matrix combined by Rz*Ry*Rx
% Rxoyx1: rotation matrix combined by Rx0*Ry*Rx1 
% Rxozx1: rotation matrix combined by Rx0*Rz*Rx1 
% Ryoxy1: rotation matrix combined by Ry0*Rx*Ry1 
% Ryozy1: rotation matrix combined by Ry0*Rz*Ry1 
% Rzoxz1: rotation matrix combined by Rz0*Rx*Rz1 
% Rzoyz1: rotation matrix combined by Rz0*Ry*Rz1 

% OUTPUT:
% eulerXYZ: euler angles around X, Y, Z axis

function eulerXYZ = decomposeRotationMat(ro, type)
r00 = ro(1,1); r01 = ro(1,2); r02 = ro(1,3);
r10 = ro(2,1); r11 = ro(2,2); r12 = ro(2,3);
r20 = ro(3,1); r21 = ro(3,2); r22 = ro(3,3);

% Type 1: Rxyz
if strcmp(type, 'Rxyz')
    if r02 < 1        
        if r02 > -1
            ry = asin(r02);
            rx = atan2(-r12,r22);
            rz = atan2(-r01,r00);
        else
            % Not a unique solution: rz - rx = atan2(r10,r11)
            ry = -pi/2;
            rx = -atan2(r10,r11);
            rz = 0;
        end
    else
        % Not a unique solution: rz + rx = atan2(r10,r11)
        ry = pi/2;
        rx = atan2(r10,r11);
        rz = 0;
    end
    
% Type 2: Rxzy
elseif strcmp(type, 'Rxzy')
    if r01 < 1
        if r01 > -1
            rz = asin(-r01);
            rx = atan2(r21,r11);
            ry = atan2(r02,r00);
        else
            % Not a unique solution: ry - rx = atan2(−r20,r22)
            rz = pi/2;
            rx = -atan2(-r20,r22);
            ry = 0;
        end
    else
        % Not a unique solution: ry + rx = atan2(−r20,r22)
        rz = -pi/2;
        rx = atan2(-r20,r22);
        ry = 0;
    end

% Type 3: Ryxz 
elseif strcmp(type, 'Ryxz')
    if r12 < 1
        if r12 > -1
            rx = asin(-r12);
            ry = atan2(r02,r22);
            rz = atan2(r10,r11);
        else
            % Not a unique solution: rz - ry = atan2(−r01,r00)
            rx = pi/2;
            ry = -atan2(-r01,r00);
            rz = 0;
        end
    else
        % Not a unique solution: rz + ry = atan2(−r01,r00)
        rx = -pi/2;
        ry = atan2(-r01,r00);
        rz = 0;
    end

% Type 4: Ryzx
elseif strcmp(type, 'Ryzx')
    if r10 < 1
        if r10 > -1
            rz = asin(r10);
            ry = atan2(-r20,r00);
            rx = atan2(-r12,r11);
        else
            % Not a unique solution: rx - ry = atan2(r21,r22)
            rz = -pi/2;
            ry = -atan2(r21,r22);
            rx = 0;
        end
    else
        % Not a unique solution: rx + ry = atan2(r21,r22)
        rz = pi/2;
        ry = atan2(r21,r22);
        rx = 0;
    end
    
% Type 5: Rzxy
elseif strcmp(type, 'Rzxy')
    if r21 < 1
        if r21 > -1
            rx = asin(r21);
            rz = atan2(-r01,r11);
            ry = atan2(-r20,r22);
        else
            % Not a unique solution: ry - rz = atan2(r02,r00)
            rx = -pi/2;
            rz = -atan2(r02,r00);
            ry = 0;
        end
    else
        % Not a unique solution: ry + rz = atan2(r02,r00)
        rx = pi/2;
        rz = atan2(r02,r00);
        ry = 0;
    end
    
% Type 6: Rzyx
elseif strcmp(type, 'Rzyx')
    if r20 < 1
        if r20 > -1
            ry = asin(-r20);
            rz = atan2(r10,r00);
            rx = atan2(r21,r22);
        else
            % Not a unique solution: rx - rz = atan2(-r12,r11)
            ry = pi/2;
            rz = -atan2(r12,r11);
            rx = 0;
        end
    else
        % Not a unique solution: rx + rz = atan2(-r12,r11)
        ry = -pi/2;
        rz = atan2(-r12,r11);
        rx = 0;
    end
end

eulerXYZ = [rx, ry, rz]*180/pi;
end





