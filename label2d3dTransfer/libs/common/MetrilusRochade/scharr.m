function [ edgeImage ] = scharr( inputImage )
%SCHARR Scharr edge detection
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
kX = [3 0 -3; 10 0 -10; 3 0 -3];
kY = [3 10 3; 0 0 0; -3 -10 -3];

%eX = conv2(double(inputImage), kX, 'same');
%eY = conv2(double(inputImage), kY, 'same');

[height, width] = size(inputImage);

edgeImage = zeros(height, width);

for y=2:height-1
    for x=2:width-1
        v1 = 3 * inputImage(y-1, x-1) - 3 * inputImage(y+1, x-1);
        if(v1 < 0)
            v1 = -v1;
        end
        v2 = 10 * inputImage(y-1, x) - 10 * inputImage(y+1, x);
        if(v2 < 0)
            v2 = -v2;
        end
        v3 = 3 * inputImage(y-1, x+1) - 3 * inputImage(y+1, x+1);
        if(v3 < 0)
            v3 = -v3;
        end
        
        temp1 = v1 + v2 + v3;
        
        h1 = 3 * inputImage(y-1, x-1) - 3 * inputImage(y-1, x+1);
        if(h1 < 0)
            h1 = -h1;
        end
        
        h2 = 10 * inputImage(y, x-1) - 10 * inputImage(y, x+1);
        if(h2 < 0)
            h2 = -h2;
        end
        
        h3 = 3 * inputImage(y+1, x-1) - 3 * inputImage(y+1, x+1);
        if(h3 < 0)
            h3 = -h3;
        end
        
        temp2 = h1 + h2 + h3;
        
        edgeImage(y, x) = sqrt(temp1*temp1 + temp2*temp2);
    end
end


%edgeImage = sqrt(eX.^2 + eY.^2);

end

