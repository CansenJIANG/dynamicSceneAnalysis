function [pairs,wcost,numpairs] = trajectoryDirectionAffinityMat(X, dim, neighborPoints)
%% Function compute the angle distance between the trajectories 
angW = double(zeros(size(X,2), size(X,2)));
rows = size(X, 1)/dim;
cols = size(X, 2);
if mod(rows,2) == 0
    len1 = rows/2;
    len2 = len1+1;
else
    len2 = round(rows/2);
    len1 = len2;
end
motThd = 0.5;
trajVec = [];
for i = 1 : size(X,2)
    trajX = reshape(X(:, i), [dim, rows])';
    dirVec = median( trajX(1:len1, :) ) - median( trajX(len2:end, :) );
    if(norm(dirVec)*len1<motThd)
        dirVec = [0, 0, -1];
    end
    trajVec = [trajVec, dirVec'];
end
for i = 1 : size(X,2)
    for j = i : size(X,2)
        x = trajVec(:, i);
        y = trajVec(:, j);
        angW(i,j) = pdist2(x',y',@angleDist);
    end
end
angW = angW + angW';
[minT,midx] = sort(angW,'ascend');

neighMat = zeros(cols,cols);
for pp = 1:cols
    neighMat(midx(1:neighborPoints,pp),pp) = 1;%[neighborPoints:-1:1];%/(0.5*neighborPoints);;
    neighMat(pp,midx(1:neighborPoints,pp)) = 1;%[neighborPoints:-1:1];%/(0.5*neighborPoints);;
end

neighMat = tril(neighMat,-1);

[nzidx] = find(neighMat>0);
[nzr,nzc] = ind2sub(size(neighMat),nzidx);
pairs = [nzr nzc]'-1;

numpairs = length(nzidx);

% wcost = ones(length(nzidx),1);
wcost = neighMat(nzidx);

[maxT,maxidx] = sort(angW,'descend');

neighMatAnti = zeros(cols,cols);
for pp = 1:cols
    neighMatAnti(maxidx(1:neighborPoints,pp),pp) = -0;%[neighborPoints:-1:1];%/(0.5*neighborPoints);;
    neighMatAnti(pp,maxidx(1:neighborPoints,pp)) = -0;%[neighborPoints:-1:1];%/(0.5*neighborPoints);;
end

neighMatAnti = tril(neighMatAnti,-1);

[nzidxAnti] = find(neighMatAnti<0);
[nzrAnti,nzcAnti] = ind2sub(size(neighMatAnti),nzidxAnti);
pairsAnti = [nzrAnti nzcAnti]'-1;

numpairsAnti = length(nzidxAnti);

% wcostAnti = 0*ones(length(nzidxAnti),1);
wcostAnti = neighMatAnti(nzidxAnti);
% 
% angW(angW<0.5*pi) = 0;
% angW = angW + triu(angW,1)';
% angW = angW./repmat(sum(angW),[size(angW, 1), 1]);
pairs = [pairs, pairsAnti];
wcost = [wcost; wcostAnti];
numpairs = numpairs + numpairsAnti;
end

function D2 = angleDist(X, Y)
D2 = atan2(norm(cross(X,Y)),dot(X,Y));
end