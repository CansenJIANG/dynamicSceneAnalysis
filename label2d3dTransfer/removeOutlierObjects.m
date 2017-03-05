%% function remove outliers of 3d objects
function [obj] = removeOutlierObjects(objCloud, label)
if size(objCloud, 1) > 100
    objCloudSeed = objCloud(1:3:end, :);
else
    objCloudSeed = objCloud;
end
showFig = 0;
[idx, dist] = rangesearch(objCloud, objCloudSeed, 1);
nnCnt = [];
for i = 1:length(idx)
    nnCnt = [nnCnt, length(idx{i})];
end
nnMax = find(nnCnt == max(nnCnt));
cloudCenter = objCloudSeed( nnMax(1), :);

if strcmp(label, 'car')
    box3d = [cloudCenter + [3.5, 3.5, 3], cloudCenter - [3.5, 3.5, 3]];
else
    box3d = [cloudCenter + [1.0, 1.0, 0.75], cloudCenter - [1.5, 1.5, 0.75]];
end
bbIdx_x = intersect(find(objCloud(:,1)<box3d(1)), ...
    find(objCloud(:,1)>box3d(4)));
bbIdx_y = intersect(find(objCloud(:,2)<box3d(2)), ...
    find(objCloud(:,2)>box3d(5)));
bbIdx_z = intersect(find(objCloud(:,3)<box3d(3)), ...
    find(objCloud(:,3)>box3d(6)));
bbIdx = intersect(bbIdx_x, bbIdx_y);
bbIdx = intersect(bbIdx, bbIdx_z);
obj = objCloud(bbIdx, :);
if showFig
 figure(1), showPointCloud(obj, [0 0 1]); hold on; 
 plot3(cloudCenter(1), cloudCenter(2), cloudCenter(3), '.r', 'MarkerSize', 30);
 plotCube(box3d, [1 0 0]);
end
end