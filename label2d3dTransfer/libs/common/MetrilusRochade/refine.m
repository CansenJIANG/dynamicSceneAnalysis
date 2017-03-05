function [ refinedCorners, refinementSuccessful ] = refine( image, initialCorners, halfSizeKernel, halfSizePatch )
%REFINE Subpixel refinement of initial corner coordinates
%
% Subpixel corner refinement
%
% [REFINEDCORNERS, REFINEMENTSUCCESSFUL] = REFINE(IMAGE,INITIALCORNERS,
% HALFSIZEKERNEL, HALFSIZEPATCH)
% Returns refined corner coordinates and if the refinement for a certain
% coordinate pair was successful. IMAGE is a double image, INITIALCORNERS a
% Nx2 vector of pixel coordinates (x,y) for N corners, HALFSIZEKERNEL the 
% half size of the smoothing kernel and HALFSIZEPATCH the half size of the 
% window in which the refinement is performed.
%
% If HALFSIZEPATCH is not set it will be set to halfSizeKernel.
%
% This function uses the file cornerfinder.m which is part of the "Caltech 
% Calibration Toolbox" by Jean-Yves Bouguet
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

if nargin == 3 
    halfSizePatch = halfSizeKernel;
end

if nargin == 4
    if halfSizePatch < halfSizeKernel
        error('The patch size must be greater than the kernel size');
    end
end

if 1
    % Use the Harris corner refiner to improve the accuracy of the initial
    % coordinates. This is particularily useful if downsampling has been
    % used in a previous step.
    harrisCorners = cornerfinder(initialCorners', image, halfSizeKernel, halfSizeKernel);
    initialCorners = harrisCorners';
end

refinedCorners = zeros(size(initialCorners));
refinementSuccessful = zeros(length(initialCorners), 1);

% Compute kernel
kernel = zeros(halfSizeKernel*2 + 1, halfSizeKernel*2 + 1);
center = halfSizeKernel + 1;
maxVal = halfSizeKernel + 1;

for n = 1:halfSizeKernel*2+1
    for m = 1:halfSizeKernel*2+1
        
        t = maxVal - sqrt((center - m)^2 + (center - n)^2);
        if t < 0
            kernel(n,m) = 0;
        else
            kernel(n,m) = t;
        end
    end
end

[height width] = size(image);

for k = 1:length(initialCorners)
    
    initialX = round(initialCorners(k,1));
    initialY = round(initialCorners(k,2));
    
    % current image patch
    
    if(initialY-halfSizePatch*2 < 1 || initialY+halfSizePatch*2 > height || initialX-halfSizePatch*2 < 1 || initialX+halfSizePatch*2 > width)
        refinedCorners(k,:) = [initialX initialY];
        refinementSuccessful(k) = 0;
        continue;
    end
    
    patch = image(initialY-halfSizePatch*2:initialY+halfSizePatch*2, initialX-halfSizePatch*2:initialX+halfSizePatch*2);
    
    % convolve kernel and patch
    filteredPatch = conv2(patch, kernel, 'valid');
    filteredPatchSize = size(filteredPatch,1);
    
    % Fit 2D polynomial
    [py, px] = meshgrid(1:filteredPatchSize, 1:filteredPatchSize);
    
    A = [ones(length(px(:)), 1) py(:) py(:).*py(:) px(:) px(:).*py(:) px(:).*px(:)];
    b = filteredPatch(:);
    p = A\b;
    
    py = p(2);
    pyy = p(3);
    px = p(4);
    pxy = p(5);
    pxx = p(6);

    saddleDetected = 0;
    
    hessDet = (4*pyy*pxx-pxy*pxy);
    if (pxy == 0)
        if(pxx == 0)
            % not a saddle point
        elseif(pyy == 0)
            % not a saddle point
        else
            if (hessDet > 0)
                % not a saddle point
            else
                saddleDetected = 1;
            end
        end
    else
        if(hessDet <= 0)
            saddleDetected = 1;
        end
    end
    
    dy1 = (py*pxy-2*pyy*px);
    dy = dy1 / hessDet;
    dx = -(px+2*pxx*dy) / pxy;
    
    if(saddleDetected == 1)
        refinedCorners(k,:) = [double(initialX) - floor(filteredPatchSize / 2) + dx - 1, double(initialY) - floor(filteredPatchSize / 2) + dy - 1];
        refinementSuccessful(k) = 1;
    else
        refinedCorners(k,:) = [initialX initialY];
        refinementSuccessful(k) = 0;
    end
end


end

