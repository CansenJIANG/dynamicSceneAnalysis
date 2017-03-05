% input: position (x, y)
%        score: scr
%   image size: (cols, rows)
%search radius: r
%
%Output: remaining features (x_, y_);

function [mainFeats, LeftIdx] = nonMaximumSuppression(feats, scr, r, kMax)
orderedIdx = 1:length(feats);
if(size(feats,1)==length(feats))
    feats = feats';
end
posScr = [feats; scr; orderedIdx]'; 
posScr = sortrows(posScr, -3);
mainFeats = posScr;
[idx, dist] = rangesearch(mainFeats(:,1:2), mainFeats(:,1:2), r);
rmIdx = [];
for k = 1:length(mainFeats)
    idxTmp = idx{k};
    featSubGrp = [mainFeats(idxTmp, :), idxTmp'];
    featSubGrp = sortrows(featSubGrp,-3);
    rmIdx = [rmIdx; featSubGrp(2:end,end)];
end
rmIdx = unique(rmIdx);
mainFeats(rmIdx,:) = []; 
if nargin>3 && kMax<length(mainFeats)
    mainFeats = sortrows(mainFeats, -3);
    mainFeats = mainFeats(1:kMax,:);
end
mainFeats = sortrows(mainFeats, 4);
LeftIdx = mainFeats(:,end)';
mainFeats = mainFeats(:,1:2)';
end