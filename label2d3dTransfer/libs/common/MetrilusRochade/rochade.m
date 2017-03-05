function [ refinedCorners, refinementSuccessful, corners ] = rochade( image, nrCornersX, nrCornersY, refinementKernelHalfSize, localRelativeThreshold, maximumHoleSizeInPercent)
%ROCHADE Summary of this function goes here
%   Detailed explanation goes here
%
% Changelog 12.02.2015
% Added automatic downsampling and downsampling threshold
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
if nargin < 4
    error('At least the first four parameters must be provided');
end

if nargin == 4
    localRelativeThreshold= 0.4;
    maximumHoleSizeInPercent = 0.01 / 100;
end

if nargin == 5
    maximumHoleSizeInPercent = 0.01 / 100;
end

downSamplingThreshold = 1000;

corners = [];
refinedCorners = [];
refinementSuccessful = [];

[height, width] = size(image);
upSampleFac = 1;

if(width < 200)
    imageInput = imresize(image, 2.0, 'bilinear');
    [height, width] = size(imageInput);
    upSampleFac = 2;
else
    imageInput = image;
end

% Downsampling for large images
downSampleFac = 1;
while (width > downSamplingThreshold)
   imageInput = imresize(imageInput, 0.5, 'nearest');
   downSampleFac = downSampleFac * 2;
   [height, width] = size(imageInput);
end


% L60-67
edgeImage = scharr(imageInput);
edgeImage = imclose(edgeImage, strel('rectangle', [3 3])); %optional

% L70-73
minImage = imerode(edgeImage, strel('rectangle', [9 9]));
maxImage = imdilate(edgeImage, strel('rectangle', [9 9]));

edgeMask = edgeImage > minImage + localRelativeThreshold * (maxImage - minImage);
edgeMask = conditionalDilation(edgeMask, 1, 5);

% L160-161
% Assume that the border pixels are not part of the checkerboard and
% therefore belong to the background
edgeMask([1 height], 1:width) = 0;
edgeMask(1:height, [1 width]) = 0;

% Fill small holes which are part of the background
edgeMask = fillHoles(edgeMask, maximumHoleSizeInPercent); %optional

% L 163-168
centerlinesThick = bwmorph(edgeMask, 'skel'); %Thick centerlines, first part of centline extraction

thinned1 = bwmorph(centerlinesThick, 'thin', 1); %Thinning, L129 in CenterlineGenerator.cs

nonLoopRemoved = removeNonLoops(thinned1, 2); %Non loop removal
noseRemoved = bwmorph(nonLoopRemoved, 'spur'); %Nose removal -> L174 in CenterlineGenerator.cs
thinnedEdgeMask = bwmorph(noseRemoved, 'thin', 1); %Rest of thinning

%Perform this step twice to go sure that there are no left-over dead-ends
%or noses
nonLoopRemoved = removeNonLoops(thinnedEdgeMask, 3);
noseRemoved = bwmorph(nonLoopRemoved, 'spur');
thinnedEdgeMask = bwmorph(noseRemoved, 'thin', 1);

%saddleCandidates = bwmorph(thinnedEdgeMask, 'branchpoints');
saddleCandidates = thinnedEdgeMask & ((filter2(ones(3), thinnedEdgeMask, 'same') > 3)); %Find graph points with 3 or more neighbors

% Find the checkerboard's connected component by flood filling the thinned
% mask
desiredCount = nrCornersX * nrCornersY;
ccStruct = bwconncomp(thinnedEdgeMask, 8);

saddleCombinationHalfSize = 2 + floor(width / 200);
saddleIncrement = max(1, floor(width / 200));
%Merge saddle points
for i = 1:4
    [combinedSaddleMask, saddleLabelImage] = combineSaddles(thinnedEdgeMask, saddleCandidates, saddleCombinationHalfSize);
    
    %Get the checkerboard candidate indices
    for k = 1:length(ccStruct.PixelIdxList)
        candidateIndices = ccStruct.PixelIdxList{k};
        numSaddles = sum(combinedSaddleMask(candidateIndices));
        if(numSaddles == desiredCount)
            [isCheckerboard, initialCornerPoints] = testForCheckerboard(saddleLabelImage, combinedSaddleMask, candidateIndices, nrCornersX, nrCornersY);
            if(isCheckerboard)
                if(upSampleFac > 1)
                    initialCornerPoints = initialCornerPoints .* (1 / upSampleFac);
                end
                
                % use original image or refinement
                if(downSampleFac > 1)
                   initialCornerPoints = downSampleFac * initialCornerPoints;
                end
                
                [refinedCorners, refinementSuccessful] = refine(image, initialCornerPoints, refinementKernelHalfSize);
                corners = initialCornerPoints;             
                return;
            end
        end
    end
      
    saddleCombinationHalfSize = saddleCombinationHalfSize + saddleIncrement;
end

end

