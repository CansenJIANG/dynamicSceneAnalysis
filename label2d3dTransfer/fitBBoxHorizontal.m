function [bbox, mu, v] = fitBBoxHorizontal(bbCloud)
mu = mean(bbCloud);
bbC_ = double(bbCloud - repmat(mu, [size(bbCloud,1), 1]));

bb2d = minBBoxHorizontal3D(bbC_(:, 1:2)');
% 
% covMat = bbC_(:,1:2)'*bbC_(:,1:2);
% [v,e]  = eig(covMat);
% bboxMax = []; bboxMin = [];
v(:, 1) = (bb2d(:, 1) - bb2d(:, 2))/norm(bb2d(:, 1) - bb2d(:, 2));
v(:, 2) = (bb2d(:, 4) - bb2d(:, 1))/norm(bb2d(:, 4) - bb2d(:, 1));
v(3,3) = 1;
for i = 1:3
    v_ = v(:, i);
    bboxMax(i) = max(bbC_*v_);
    bboxMin(i) = min(bbC_*v_);
end
bboxMin(end) = bboxMin(end) - 0.2;

% if strcmp(label,'car')
%     bboxMax = bboxMax + [];
%     bboxMin = bboxMin;
% end
% bboxMax(3) = max(bbC_(:,3)); bboxMin(3) = min(bbC_(:,3));
bbox(:,7) = bboxMax(1)*v(:, 1) + bboxMax(2)*v(:, 2) + bboxMax(3)*v(:, 3);
bbox(:,8) = bboxMax(1)*v(:, 1) + bboxMax(2)*v(:, 2) + bboxMin(3)*v(:, 3);
bbox(:,3) = bboxMax(1)*v(:, 1) + bboxMin(2)*v(:, 2) + bboxMax(3)*v(:, 3);
bbox(:,5) = bboxMax(1)*v(:, 1) + bboxMin(2)*v(:, 2) + bboxMin(3)*v(:, 3);
bbox(:,2) = bboxMin(1)*v(:, 1) + bboxMax(2)*v(:, 2) + bboxMax(3)*v(:, 3);
bbox(:,6) = bboxMin(1)*v(:, 1) + bboxMax(2)*v(:, 2) + bboxMin(3)*v(:, 3);
bbox(:,1) = bboxMin(1)*v(:, 1) + bboxMin(2)*v(:, 2) + bboxMax(3)*v(:, 3);
bbox(:,4) = bboxMin(1)*v(:, 1) + bboxMin(2)*v(:, 2) + bboxMin(3)*v(:, 3);
bbox = bbox' + repmat(mu, [8,1]);

% figure, showPointCloud(bbC_, [0 0 1]); hold on;
% plotCube(bbox');
end