% function for bisection search algorithm
% function use to search the closest point in vec using query point
% query: is the value to search
% vec: is the vector contains a set of values

function idx = bisectionSearch(query, vec)
idxleft = 1; idxright = length(vec); 
% leftHandle = []; rightHandle = [];
% figure, plot(vec); hold on; 
while abs(idxleft-idxright)>1
%     delete(leftHandle); delete(rightHandle);
    midIdx = floor(0.5*(idxleft+idxright));
    if( query > vec(midIdx))
        idxleft = midIdx;
    else
        idxright = midIdx;
    end
%     leftHandle = plot(idxleft, vec(idxleft), '+r');
%     rightHandle = plot(idxright, vec(idxright), 'or');
%     drawnow;
end
if idxright ~= idxleft
    if query - vec(idxleft) > vec(idxright)-query
        idx = idxright;
    else
        idx = idxleft;
    end
else
    idx = idxright;
end
end