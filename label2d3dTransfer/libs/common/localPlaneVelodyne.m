function [cloudNoGnd, gndCloud] = localPlaneVelodyne(cloud, winSize, gndThd, inlierThd)

%% function detect the ground elements

if 0
    load('testdata.mat');
    inlierThd = 0.1;
    gndThd = -1.0;
    winSize = 3;
end
if size(cloud, 1) == 3
    cloud = cloud';
end
[~, ~, planeIdx] = ransacfitplane(cloud',0.2);


max_X = max(cloud(:,1)); min_X = min(cloud(:,1));
max_Y = max(cloud(:,2)); min_Y = min(cloud(:,2));

%% construct the KD-tree
cloudNoPlane = cloud;  cloudNoPlane(planeIdx, :) = [];
% Build KD-tree for fast searching
MdlKDT = createns(cloud(:,1:2),'NSMethod','kdtree','Distance','cityblock');
% figure, plot(cloud(:,1),cloud(:,2),'.r'); hold on;
% Define the subpatches
[x, y] = meshgrid([min_X:winSize:max_X],[min_Y:winSize:max_Y]);
centers = [x(:), y(:)];
% plot(centers(:,1), centers(:,2), '.b');
% Search the point within the subpatches
Idx = rangesearch(MdlKDT,centers,winSize,'Distance','cityblock');

gndIdx = []; localPlaneVar = inlierThd;
% consider the lowest point as ground
for i=1:length(Idx)
    subIdx = Idx{i};
    if(length(subIdx)>0)
        if(length(subIdx)<4)
            gndIdx = [gndIdx, subIdx];
        else
            subSet = cloud(subIdx,3);
            %             [~, ~, localIdx] = ransacfitplane(subSet',0.1);
            %             gndZ = mean(subSet(localIdx,end));
            %             localIdx = unique([localIdx; find(subSet(:,end)<gndZ)]);
            %             gndIdx = [gndIdx, subIdx(localIdx)];
            minZ = min(subSet);
            if(minZ<gndThd)
                if(minZ<-2)
                    gndIdx = [gndIdx, subIdx(find(subSet==minZ))];
                    subIdx(find(subSet==minZ)) = [];
                    subSet(subSet==minZ) = [];
                    minZ = min(subSet);
                end
                newGndIdx = find(subSet< max(minZ+localPlaneVar, -1.5));
                gndIdx = [gndIdx, subIdx(newGndIdx)];
            end
        end
    end
end
% Remove center noisy points
% outlierIdx = rangesearch(MdlKDT,[-4.26, 0.074],3.2,'Distance','cityblock');
gndIdx = unique([gndIdx, planeIdx']);
gndCloud = cloud(gndIdx,:); gndCloud = gndCloud';
cloudNoGnd = cloud; cloudNoGnd(gndIdx,:) = []; cloudNoGnd = cloudNoGnd';
% figure, showPointCloud(cloudNoGnd'); title('cloudNoGnd');
% figure, showPointCloud(gndCloud');title('gndPts');
end
