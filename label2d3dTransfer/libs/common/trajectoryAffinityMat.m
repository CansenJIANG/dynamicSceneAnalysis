%% Construct the Affinity Matrix between Feature Trajectories
% D = pdist2(X,Y)
% D = pdist2(X,Y,distance)
% D = pdist2(X,Y,'minkowski',P)
% D = pdist2(X,Y,'mahalanobis',C)
% D = pdist2(X,Y,distance,'Smallest',K)
% D = pdist2(X,Y,distance,'Largest',K)
% [D,I] = pdist2(X,Y,distance,'Smallest',K)
% [D,I] = pdist2(X,Y,distance,'Largest',K)
function [W] = trajectoryAffinityMat(X, dim, distMetric)
if nargin<3
    distMetric = 'minkowski'; 
end
if nargin<2
    dim = 3;
end
X_ = [];
for i = 1:size(X,1)/dim
X_ = [X_; reshape(X(dim*(i-1)+1:dim*i,:), [1, dim*size(X,2)])];
end
X_ = X_ - repmat(X_(1,:), [size(X_,1), 1]);
X_ = X_(2:end,:);

% distMetric = 'euclidean'; X
% distMetric = 'squaredeuclidean'; 
% distMetric = 'seuclidean';
% distMetric = 'cityblock';
% distMetric = 'minkowski';
% distMetric = 'chebychev'; 
% distMetric = 'mahalanobis'; X
% distMetric = 'cosine'; X
% distMetric = 'correlation'; X
% distMetric = 'spearman'; X
% distMetric = 'hamming'; X
% distMetric = 'jaccard'; X
% distMetric = 'chisq'; X
W = double(zeros(size(X,2), size(X,2)));
for i = 1 : size(X,2)
    for j = i : size(X,2)
        x = double(X_(:, dim*(i-1)+1:dim*i));
        y = double(X_(:, dim*(j-1)+1:dim*j));
        di = pdist2(x',y',distMetric);
        W(i,j) = log(1+1/norm(diag(di)));
    end
end
W(isinf(W)) = 0; W(isnan(W)) = 0; 
W = W' + W;
W = W + eye(size(W))*10e-6;
W = W./repmat(sum(W),[size(W, 1), 1]);
end
