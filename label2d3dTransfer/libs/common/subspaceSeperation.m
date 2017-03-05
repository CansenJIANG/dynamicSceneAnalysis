% function to re-organize data arrangement according to the clustering
% result from SSC
% function cloud_ = subspaceSeperation(cloud, classIdx)
% cloud: unsorted point cloud in a video sequence
% classIdx: index of corresponding subspaces
% cloud_: groupped data according to their subspace
% nb: number of subspace
% size: size of each subspace, number of elements in the subspace

function [cloud_, nb, classSize]= subspaceSeperation(cloud, classIdx)

% group the data
cloud = [cloud;classIdx'];
cloud_ = sortrows(cloud', size(cloud',2));
cloud_ = cloud_';
cloud_(end,:) = [];
% get number of classes
nb = length(unique(classIdx));

% get size of each subspace
for i = 1:nb
    classSize(i) = length( find(classIdx == i) );
end

end