%% Function to compute OF speed of the features
%% Input: 
%% vx: x direction opitcal flow
%% vy: y direction optical flow
%% trkFeat: features position in image
%% Output:
%% trkOF: optical flow values of trkFeat
%% trkFeat: new position of features in next frame of image
function [trkFeat, trkOF] = getFeatureOFvelocity(trkFeat, vx, vy)
trkOF = zeros(size(trkFeat));
for i = 1:length(trkFeat)
    px = floor(trkFeat(i,1));
    py = floor(trkFeat(i,2));
    % Get features' OF and their new potions
    if(trkFeat(i,1)==px && trkFeat(i,2)==py)
        trkFeat(i,1) = trkFeat(i,1)+vx(py, px);
        trkFeat(i,2) = trkFeat(i,2)+vy(py, px);
    else
        % features move out of the image
        if(px<1 || px>size(vx,2)-1 ||py<1 || py>size(vx,1)-1)
            trkFeat(i,:) = [1, 1];
            outFoV = [outFoV; i];
            continue;
        end
        % get OF of 4 neighbouring pixels
        Neigh4 = [px, py; px+1, py; px, py+1; px+1, py+1]; % 4 neighbourhood
        % compute OF using 4 neighbouring OF values
        for j = 1:4
            if(max(Neigh4(:,2))>size(vx,1) | max(Neigh4(:,1))>size(vx,2))
                display(max(Neigh4(:,2)));
                display(max(Neigh4(:,1)));
            end
            Neigh4vx(j) = vx(Neigh4(j,2), Neigh4(j,1));
            Neigh4vy(j) = vy(Neigh4(j,2), Neigh4(j,1));
        end
        weight4 = repmat(trkFeat(i,:),[4, 1]) - Neigh4;
        weight4 = weight4.*weight4;
        weight4 = 1./sum(weight4');
        weight4 = weight4/sum(weight4);
        deltaX = weight4*Neigh4vx';
        deltaY = weight4*Neigh4vy';
        trkFeat(i,1) = trkFeat(i,1) + deltaX;
        trkFeat(i,2) = trkFeat(i,2) + deltaY;
        trkOF(i,1)  = deltaX;
        trkOF(i,2)  = deltaY;
    end
    
    % features move out of the image
    if(trkFeat(i,1)<1 || trkFeat(i,1)>size(vx,2)-1 ||...
            trkFeat(i,2)<1 || trkFeat(i,2)>size(vx,1)-1)
        trkFeat(i,:) = [1, 1];
        continue;
    end
end