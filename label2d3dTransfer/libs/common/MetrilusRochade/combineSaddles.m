function [ combinedSaddleMask saddleLabelImage ] = combineSaddles(edges, saddleMask, saddleCombinationHalfSize)
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
%Remove saddles in margin regions in order to eliminate bounds checks in
%region growing
combinedSaddleMask = saddleMask;
combinedSaddleMask(1:saddleCombinationHalfSize,:) = false;
combinedSaddleMask(end+1-saddleCombinationHalfSize:end, :) = false;
combinedSaddleMask(:, 1:saddleCombinationHalfSize) = false;
combinedSaddleMask(:, end+1-saddleCombinationHalfSize) = false;

[height width] = size(edges);

saddleLabelImage = zeros(height, width);

visited = zeros(height, width);
segmentNumber = 1;
totalSaddleCount = 0;

for y=1:height
    for x=1:width
        if(combinedSaddleMask(y,x))
            currentSaddleList = java.util.LinkedList;
            csi = currentSaddleList.iterator;
            currentSaddleList.add([x y]);        
            saddleLabelImage(y, x) = segmentNumber;
            saddleMask(y,x) = false;            
            while(currentSaddleList.size > 0)
                newSaddleList = java.util.LinkedList;
                csi = currentSaddleList.iterator;
                while(csi.hasNext())
                    s = csi.next();
                    sX = s(1);
                    sY = s(2);
                    totalSaddleCount = totalSaddleCount + 1;
                    currentPointNeighborhoodList = java.util.LinkedList;           
                    currentPointNeighborhoodList.add([sX sY]);
                    visited(sY, sX) = totalSaddleCount;
                    for radiusCount = 1:saddleCombinationHalfSize
                        newNeighborhoodList = java.util.LinkedList;
                        cnli = currentPointNeighborhoodList.iterator;
                        while (cnli.hasNext())
                            p = cnli.next();
                            pX = p(1);
                            pY = p(2);
                            for j = -1:1
                                for i = -1:1
                                    yj = pY + j;
                                    xi = pX + i;
                                    if (edges(yj, xi) && visited(yj, xi) ~= totalSaddleCount) % go only in unvisited points
                                        visited(yj, xi) = totalSaddleCount;
                                        newNeighborhoodList.add([xi yj]);
                                        if (combinedSaddleMask(yj, xi))
                                            combinedSaddleMask(yj, xi) = false;
                                            saddleLabelImage(yj, xi) = segmentNumber;
                                            newSaddleList.add([xi yj]);
                                        end
                                    end
                                end
                            end
                        end
                        currentPointNeighborhoodList = newNeighborhoodList;
                    end
                end
                currentSaddleList = newSaddleList;
            end
            
            %restore the saddle
            combinedSaddleMask(y, x) = true;
            %Increment the saddle cluster label
            segmentNumber = segmentNumber + 1;
        end
    end
end

end