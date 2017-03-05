% region growing 3D
% input: point cloud
% input: seed
% input: searching radius
% output: segments
% output: scene without segment
% output: index corresponding to segment
function [objSeg, segIdx, bbox, bbAxis] = regionGrow3dPointCloud(scene, seed, label)
segIdx = [];
[bbox, mu, bbAxis] = fitBBoxHorizontal(seed);
if 0
    figure(1), showPointCloud(seed, [0 0 1]); hold on;
    plotCube(bbox, [1 0 0]);
end
bboxAxis = (bbox*bbAxis);
sizeObj = [max(bboxAxis(:,1)) - min(bboxAxis(:,1)), ...
    max(bboxAxis(:,2)) - min(bboxAxis(:,2)), ...
    max(bboxAxis(:,3)) - min(bboxAxis(:,3))];
if strcmp(label, 'bicycle') || strcmp(label, 'bench') || strcmp(label, 'motorbike') || ...
        strcmp(label, 'umbrella')
    objSeg = seed;
    return;
end
segNb = size(seed, 1); maxIter = 10;
seed = randomSampleSeeds(segNb, seed, 1.0);
iter = 0; objSegIdx = []; objSeg = [];
if strcmp(label, 'car') || strcmp(label, 'traffic_light') || strcmp(label, 'bus')
    ra = 0.5;
else
    ra = 0.25;
end

while(iter<maxIter)
    [idx, dist] = rangesearch(scene, seed, ra);
    for i=1:length(idx)
        %         seed = [seed; scene(idx{i}, :)];
        objSegIdx = [objSegIdx, idx{i}];
    end
    objSegIdx = unique(objSegIdx);
    objSeg = scene(objSegIdx, :);
    if( segNb==size(objSeg, 1) )
        break;
    end
    segNb = size(objSeg, 1);
    iter = iter +1;
    seed = randomSampleSeeds(segNb, objSeg);
end
[bbox, mu, bbAxis] = fitBBoxHorizontal(objSeg);
if 0
    figure(1), showPointCloud(seed, [1 0 0]); hold on;
    plotCube(bbox, [0 0 1]);
end
end

function seedSam = randomSampleSeeds(segNb, seed, ratio)
if nargin<3
    ratio = 0.2;
end
% This function is naive, need to be improved.
if segNb > 50
    selectIdx = randi([1, segNb], [max(round(segNb*ratio),50), 1]);
    seedSam = seed(selectIdx, :);
else
    seedSam = seed;
end
end