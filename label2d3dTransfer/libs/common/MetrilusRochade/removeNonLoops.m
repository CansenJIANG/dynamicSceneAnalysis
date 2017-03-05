function [ edges ] = removeNonLoops( inputEdges, margin )
%REMOVENONLOOPS Summary of this function goes here
%   Detailed explanation goes here
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
[height, width] = size(inputEdges);

edges = inputEdges;

for y = margin+1:height-margin
    for x = margin+1:width-margin

        % The number of edge pixels in the neighborhood is given by the sum
        % of the window
        if(inputEdges(y, x))
            nb = sum(sum(inputEdges(y-1:y+1, x-1:x+1))) - 1;

            if nb == 0

                % No neighbors
                edges(y,x) = 0;
            elseif nb == 1
                % Dead end
                edges = unFillLoopCondition(edges, y, x);
            end
        end
        
    end
end

end

function edges = unFillLoopCondition(inputEdges, sy, sx)

edges = inputEdges;
[height, width] = size(edges);

nextX = sx;
nextY = sy;

while 1
    % number of neighbors
    nb = 0;
    
    x = nextX;
    y = nextY;
    
    for j = [-1, 0, 1]
        for k = [-1, 0, 1]
            if(j == 0 && k == 0)
                continue;
            end
            

            
            yj = y + j;
            xk = x + k;
            
            % current coordinate within the image && edge ~= 0
            if (yj > 0 && yj <= height && xk > 0 && xk <= width && edges(yj, xk) ~= 0)
                nb = nb + 1;
                nextX = xk;
                nextY = yj;
            end
            
        end
    end

    if nb == 0
        edges(y,x) = 0;
        return;
    elseif nb == 1;
        edges(y,x) = 0;
    else
        return;
    end
end

end


