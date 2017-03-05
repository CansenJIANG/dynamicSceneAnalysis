function [ m ] = fillHoles(mask, percentage)
%FILLHOLES Fills small holes in a mask image
%
% fileHoles(inputImage, percentage)
%
% Removes holes from a mask which are smaller than a given percentage of
% the whole image size. The percentage is given as a number between 0 and
% 1.
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
m = mask;
[height, width] = size(mask);

smallThreshold = floor(height*width*percentage);

bg = imfill(mask, 1, 4);
holeMask = ~bg | mask;
cc = bwconncomp(holeMask, 4);
ccIdxLength = cellfun('length',cc.PixelIdxList);

for k = 1:length(ccIdxLength)
    
    if ccIdxLength(k) < smallThreshold
        holeIdx = cell2mat(cc.PixelIdxList(k));
        m(holeIdx) = 0;
    end
end


end
