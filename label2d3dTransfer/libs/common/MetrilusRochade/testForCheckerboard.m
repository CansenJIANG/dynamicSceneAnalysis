function [ isCheckerboard initialCorners] = testForCheckerboard(saddleLabelImage, combinedSaddleMask, candidateIndices, checkerboardWidth, checkerboardHeight)
%
% - - - - - - 
% This file is part of ROCHADE.
% 
% ROCHADE is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ROCHADE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with ROCHADE.  If not, see <http://www.gnu.org/licenses/>.
%
% Copyright 2014 Simon Placht, Peter Fürsattel, Metrilus GmbH,
% www.metrilus.de
isCheckerboard = false;
initialCorners = zeros(checkerboardHeight*checkerboardWidth, 2);

[height width] = size(saddleLabelImage);

checkMaskCandidate = false(height, width);
checkMaskCandidate(candidateIndices) = true;

saddleMaskOfCandidate = checkMaskCandidate & combinedSaddleMask;
saddleLabels = saddleLabelImage(saddleMaskOfCandidate); %Now we have the checkerboardWidth x checkerboardHeight labels

% Replace the labels in the candiates, so that they run from 1 to number
% corners

indexTable = zeros(1, max(saddleLabels));
indexTable(saddleLabels) = 1:(checkerboardWidth * checkerboardHeight);

allSaddleMask = (saddleLabelImage > 0) & checkMaskCandidate;

saddleIndices = find(allSaddleMask);

saddleLabelImage(saddleIndices) = indexTable(saddleLabelImage(saddleIndices));

%set containing the neighbors for each saddle label
labelsOfSaddleNeighbors = java.util.Vector;
labelsOfSaddleNeighbors.setSize(length(saddleLabels));

for i=0:length(saddleLabels)-1
    neighborLabels = java.util.TreeSet;
    labelsOfSaddleNeighbors.setElementAt(neighborLabels, i);
end

initialSaddleX = zeros(1, length(saddleLabels));
initialSaddleY = zeros(1, length(saddleLabels));
initialSaddleSum = zeros(1, length(saddleLabels));

for i=1:length(saddleIndices)
    idx = saddleIndices(i);
    saddleLabel = saddleLabelImage(idx);
    [y x] = ind2sub(size(checkMaskCandidate), idx);
    
    initialSaddleX(saddleLabel) = initialSaddleX(saddleLabel) + x;
    initialSaddleY(saddleLabel) = initialSaddleY(saddleLabel) + y;
    initialSaddleSum(saddleLabel) = initialSaddleSum(saddleLabel) + 1;
    
    %Now find the neighbors of the saddle
    for n=-1:1
        for m=-1:1
            yn = y + n;
            xm = x + m;
            if(checkMaskCandidate(yn, xm) & saddleLabelImage(yn, xm) == 0)
                nextSaddle = -1;
                
                %Unfill to next saddle
                checkMaskCandidate(yn, xm) = false;
                xNext = xm;
                yNext = yn;
                pointsAvailable = true;
                while(pointsAvailable)
                    nextPointFound = false;
                    for j=-1:1
                        for i =-1:1
                            yj = yNext + j;
                            xi = xNext + i;
                            if( yj > 0  & yj <= height & xi > 0 & xi <= width & checkMaskCandidate(yj, xi) & saddleLabelImage(yj, xi) ~= saddleLabel)
                                if(saddleLabelImage(yj, xi) > 0)
                                    nextSaddle = saddleLabelImage(yj, xi);
                                    break;
                                else
                                    checkMaskCandidate(yj, xi) = false;
                                    nextPointFound = true;
                                    xNext = xi;
                                    yNext = yj;
                                    break;
                                end
                            end
                        end
                        if(nextPointFound | nextSaddle > 0)
                            break;
                        end
                        if(j == 1)
                            pointsAvailable = false;
                        end
                    end
                    if(nextSaddle > 0)
                        break;
                    end
                end
                
                if(nextSaddle > 0)
                    labelsOfSaddleNeighbors.elementAt(saddleLabel-1).add(nextSaddle);
                    labelsOfSaddleNeighbors.elementAt(nextSaddle-1).add(saddleLabel);
                end
            end
        end
    end
end

initialSaddleX = initialSaddleX ./ initialSaddleSum;
initialSaddleY = initialSaddleY ./ initialSaddleSum;

twoNeighborSaddles = 0;
threeNeighborSaddles = 0;
fourNeighborSaddles = 0;

twoNeighborSaddleLabels = [];

originalNeighborCount = zeros(1, length(saddleLabels));

% count two, three, and four neighbor saddles
for i=0:length(saddleLabels)-1
    numNeighbors = labelsOfSaddleNeighbors.elementAt(i).size();
    originalNeighborCount(i+1) = numNeighbors;
    if(numNeighbors == 2)
       twoNeighborSaddles = twoNeighborSaddles + 1;
       twoNeighborSaddleLabels = [twoNeighborSaddleLabels, (i+1)];
    elseif (numNeighbors == 3)
       threeNeighborSaddles = threeNeighborSaddles + 1;
    elseif (numNeighbors == 4)
        fourNeighborSaddles = fourNeighborSaddles + 1;
    else
        return;
    end
end

 desiredNumberOf2NeighborSaddles = 4;
 desiredNumberOf3NeighborSaddles = 2 * checkerboardHeight + 2 * checkerboardWidth - 8;
 desiredNumberOf4NeighborSaddles = checkerboardWidth * checkerboardHeight - desiredNumberOf2NeighborSaddles - desiredNumberOf3NeighborSaddles;
 
 if( (twoNeighborSaddles ~= desiredNumberOf2NeighborSaddles) | (threeNeighborSaddles ~= desiredNumberOf3NeighborSaddles) | (fourNeighborSaddles ~= desiredNumberOf4NeighborSaddles))
     return;
 end

cornerLabels = zeros(checkerboardHeight, checkerboardWidth);

%Now grow from one two neighborsaddle until the next threeNeigbhorsaddle is
%reached

startLabel = twoNeighborSaddleLabels(1);

paths = cell(1, 2);

next = [labelsOfSaddleNeighbors.elementAt(startLabel-1).first(), labelsOfSaddleNeighbors.elementAt(startLabel-1).last()];

paths{1} = [startLabel next(1)];
labelsOfSaddleNeighbors.elementAt(startLabel-1).remove(next(1));
labelsOfSaddleNeighbors.elementAt(next(1)-1).remove(startLabel);

paths{2} = [startLabel next(2)];
labelsOfSaddleNeighbors.elementAt(startLabel-1).remove(next(2));
labelsOfSaddleNeighbors.elementAt(next(2)-1).remove(startLabel);

for i=1:2
    while(next(i) > 0)
        if(labelsOfSaddleNeighbors.elementAt(next(i)-1).size() ~= 2)
            break;
        end     
        neighbors = [labelsOfSaddleNeighbors.elementAt(next(i)-1).first(), labelsOfSaddleNeighbors.elementAt(next(i)-1).last()];
        nextIdx = -1;
        for j=1:length(neighbors)
            if(labelsOfSaddleNeighbors.elementAt(neighbors(j)-1).size() == 2 | labelsOfSaddleNeighbors.elementAt(neighbors(j)-1).size() == 3)
                labelsOfSaddleNeighbors.elementAt(next(i)-1).remove(neighbors(j));
                labelsOfSaddleNeighbors.elementAt(neighbors(j)-1).remove(next(i));
                nextIdx = neighbors(j);
                paths{i} = [paths{i}, nextIdx];
                break;
            end
        end
          
        next(i) = nextIdx;
    end
end

if(length(paths{1}) == checkerboardWidth)
    if(length(paths{2}) == checkerboardHeight)
        cornerLabels(1, 1:end) = paths{1};
        cornerLabels(1:end, 1) = paths{2};
        path = paths{1};
        for i=2:length(path)
            neighbor = labelsOfSaddleNeighbors.elementAt(path(i)-1).first();
            labelsOfSaddleNeighbors.elementAt(path(i)-1).remove(neighbor);
            labelsOfSaddleNeighbors.elementAt(neighbor-1).remove(path(i));
        end       
    end
else
    if(length(paths{2}) == checkerboardWidth)
        if(length(paths{1}) == checkerboardHeight)
            cornerLabels(1, 1:end) = paths{2};
            cornerLabels(1:end, 1) = paths{1};
            path = paths{2};
            for i=2:length(path)
                neighbor = labelsOfSaddleNeighbors.elementAt(path(i)-1).first();
                labelsOfSaddleNeighbors.elementAt(path(i)-1).remove(neighbor);
                labelsOfSaddleNeighbors.elementAt(neighbor-1).remove(path(i));
            end  
        end
    end
end

if(cornerLabels(1) == 0)
    return;
end

for y=2:checkerboardHeight
    xCount = 1;
    startLabel = cornerLabels(y, xCount);
    next = labelsOfSaddleNeighbors.elementAt(startLabel-1).first();
    labelsOfSaddleNeighbors.elementAt(startLabel-1).remove(next);
    labelsOfSaddleNeighbors.elementAt(next-1).remove(startLabel);
    xCount = xCount + 1;
    cornerLabels(y, xCount) = next;
    while(next > 0)
        neighbors = [];
        if(y < checkerboardHeight)
            if(labelsOfSaddleNeighbors.elementAt(next-1).size() ~= 2)
                if(labelsOfSaddleNeighbors.elementAt(next-1).size() == 1)
                    %Remove neighbor
                    neighbor = labelsOfSaddleNeighbors.elementAt(next-1).first();
                    labelsOfSaddleNeighbors.elementAt(next-1).remove(neighbor);
                    labelsOfSaddleNeighbors.elementAt(neighbor-1).remove(next);
                end
                break;
            end
            neighbors = [labelsOfSaddleNeighbors.elementAt(next-1).first(), labelsOfSaddleNeighbors.elementAt(next-1).last()];
        else
            if(labelsOfSaddleNeighbors.elementAt(next-1).size() ~= 1)
                break;
            end
            neighbors = [labelsOfSaddleNeighbors.elementAt(next-1).first()];
        end
        
        nextIdx = -1;
        for j=1:length(neighbors)      
            numNeighbors = labelsOfSaddleNeighbors.elementAt(neighbors(j)-1).size();
            if(numNeighbors == 2 || (numNeighbors == 3 && originalNeighborCount(neighbors(j)) == 4) || (y == checkerboardHeight && numNeighbors == 1))
                nextIdx = neighbors(j);
                xCount = xCount +1;
                if(xCount > checkerboardWidth)
                    return;
                end
                cornerLabels(y, xCount) = nextIdx;         
                break;
            end
        end
        
        for j=1:length(neighbors)
            labelsOfSaddleNeighbors.elementAt(next-1).remove(neighbors(j));
            labelsOfSaddleNeighbors.elementAt(neighbors(j)-1).remove(next);
        end
          
        next = nextIdx;
    end
end

xTL = initialSaddleX(cornerLabels(1,1));
yTL = initialSaddleY(cornerLabels(1,1));

xTR = initialSaddleX(cornerLabels(1,end));
yTR = initialSaddleY(cornerLabels(1,end));

xBL = initialSaddleX(cornerLabels(end, 1));
yBL = initialSaddleY(cornerLabels(end, 1));

normalY = xTR - xTL;
normalX = yTL - yTR;

if( ((xBL - xTL)*normalX + (yBL - yTL)*normalY) < 0)
    cornerLabels = flip(cornerLabels, 2);
end

idx = 1;
for y=1:checkerboardHeight
    for x=1:checkerboardWidth
        initialCorners(idx, 1) = initialSaddleX(cornerLabels(y, x));
        initialCorners(idx, 2) = initialSaddleY(cornerLabels(y, x));
        idx = idx + 1;
    end
end

isCheckerboard = true;
return;

end