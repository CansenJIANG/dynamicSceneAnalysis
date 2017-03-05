%%[dist] = distances2vectors(vec1, vec2)
%% compute distance bwt two vectors
function [dist] = distances2vectors(vec1, vec2)
if(size(vec1,2)~=length(vec1))
    vec1 = vec1';
end
if(size(vec2,2)~=length(vec2))
    vec2 = vec2';
end
vecDist = vec1 - vec2;
dist = sqrt( sum(vecDist.*vecDist) )';
end