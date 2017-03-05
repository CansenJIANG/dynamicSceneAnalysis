function [bbCloud_img, objCloud] = preciseObjectLocalization(cloudFoV, bbox, P_velo_to_img, bbAxis)
for i = 1:3
    v = bbAxis(:, i);
    bboxMax(i) = max(bbox*v);
    bboxMin(i) = min(bbox*v);
    projAxis(:, i) = cloudFoV*v;
end

bbIdx_x = intersect(find(projAxis(:,1)<bboxMax(1)), ...
    find(projAxis(:,1)>bboxMin(1)));
bbIdx_y = intersect(find(projAxis(:,2)<bboxMax(2)), ...
    find(projAxis(:,2)>bboxMin(2)));
bbIdx_z = intersect(find(projAxis(:,3)<bboxMax(3)), ...
    find(projAxis(:,3)>bboxMin(3)));
bbIdx = intersect(bbIdx_x, bbIdx_y);
bbIdx = intersect(bbIdx, bbIdx_z);
objCloud = cloudFoV(bbIdx, :);
[bbCloud_img, bbCloud_depth] = project(objCloud, P_velo_to_img);

end