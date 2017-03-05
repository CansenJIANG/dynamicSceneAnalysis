
% function estimate the rigid transformation error
% function err = rigidTransformError(X, Y, T)
% X: 3xN, N 3-D points (N>=3)
% Y: 3xN, N 3-D points (N>=3)
% T: 4x4 transformation matrix in homogeneous coordinate

function err = rigidTransformError(X, Y, T)
% transform data
Y_ = [Y; ones(1,length(Y))];
Y_ = T*Y_;
% Normalize transformed data
Y_(1,:) = Y_(1,:)./Y_(end,:);
Y_(2,:) = Y_(2,:)./Y_(end,:);
Y_(3,:) = Y_(3,:)./Y_(end,:);
Y_(end,:) = Y_(end,:)./Y_(end,:);
% estimate l2 norm of error matrix
err = norm(X -Y_(1:3,:) );
end