function bbox = fitBBoxPCA(bbCloud)
mu = mean(bbCloud);
bbC_ = bbCloud - repmat(mu, [size(bbCloud,1), 1]);
covMat = bbC_'*bbC_;
[v,e]  = eig(covMat);
bboxMax = []; bboxMin = [];
for i = 1:3
    v_ = v(:, i);
    bboxMax(i) = max(bbC_*v_);
    bboxMin(i) = min(bbC_*v_);
end

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