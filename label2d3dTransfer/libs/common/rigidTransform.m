% function for rigid transformation based on rotation angles and
% translation vector.
% function obj_ = rigidTransform(obj, ro, tr)
% obj: 3D object to be transformed
% ro: rotation angles along X, Y, Z axis
% tr: translation vector along X, Y, Z axis

function obj_ = rigidTransform(obj, ro, tr)

    % convert rotation angle to rotation matrix
    R = ang2RotMat(ro);
    
    % Transformation matrix
    T = [R, tr'; 0, 0, 0, 1];
    
    obj_ = T* [obj';ones(1,length(obj))];
    
    % Normalize transformed data
    obj_(1,:) = obj_(1,:)./obj_(end,:);
    obj_(2,:) = obj_(2,:)./obj_(end,:);
    obj_(3,:) = obj_(3,:)./obj_(end,:);
    obj_ = obj_(1:3,:)';
end