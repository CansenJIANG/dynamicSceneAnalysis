function [ dilatedMask ] = conditionalDilation( inputMask, halfSize, minNrSetPixels )
%CONDITIONALDILATION Requires a min number of pixels to be set for dilation
%
% Computes a dilated image if more than a certain number of pixels is set within
% the kernel. The kernel size is defined by its half size.
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
if ~isa(inputMask, 'logical')
    error('The input mask must be of type logical');
end

dilatedMask = inputMask;

[height, width] = size(image);
for y = 1:height
    for x = 1:width
        
        if dilatedMask(y,x) == 0
            setPixels = 0;
        
            for yy = y-halfSize:y+halfSize
                
                if (yy < 1 || yy > height)
                    continue;
                end
                
                for xx = x-halfSize:x+halfSize
                    
                    if (xx<1 || x>width)
                        continue;
                    end
                    
                    if dilatedMask ~= 0
                        setPixels = setPixels+1;
                    end
                    
                end
            end %end kernel y-loop
            
            if(setPixels > minNrSetPixels)
                dilatedMask(y,x) = 1;
            end

        end %end mask==0
    end
end

end

