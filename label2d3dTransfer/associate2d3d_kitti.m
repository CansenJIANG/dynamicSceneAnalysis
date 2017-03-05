function [cloud3dFoV, cloud3dFoV_Color, fovProj3DUnique] =...
    associate2d3d_kitti(leftImgRef, cloud3d, P_velo_to_img)
[cloud_img, cloud_depth] = project(cloud3d, P_velo_to_img);

% Lets validate only those 3D and projected points which are within image
% Image size
Ix = size(leftImgRef,2); Iy = size(leftImgRef,1);
backIdx = find(cloud_depth<=0);
cloud_img(backIdx, :) = [];

cloud_img = cloud_img';
fovList = cloud_img(1,:)>0.5 & cloud_img(2,:)>0.5 &...
    cloud_img(1,:)< (Ix- 0.5) & cloud_img(2,:)<(Iy- 0.5);
cloud3dFoV = cloud3d;
cloud3dFoV(backIdx, :) = [];
cloud3dFoV = cloud3dFoV(fovList, :);

%% Associate to the image color information
fovProj3D  = cloud_img(:,fovList);
fovProj3D = round(fovProj3D);
[idxUnique,ia,ic] = unique(fovProj3D','rows');
fovProj3DUnique = fovProj3D(:, ia);
cloud3dFoV = cloud3dFoV(ia,:);

projImg = []; cloud3dFoV_Color = [];
projImgR = leftImgRef(:,:,1);
projImgG = leftImgRef(:,:,2);
projImgB = leftImgRef(:,:,3);
cloud3dFoV_Color(:,1) = projImgR(sub2ind(size(projImgR), idxUnique(:, 2), idxUnique(:,1)));
cloud3dFoV_Color(:,2) = projImgG(sub2ind(size(projImgG), idxUnique(:, 2), idxUnique(:,1)));
cloud3dFoV_Color(:,3) = projImgB(sub2ind(size(projImgB), idxUnique(:, 2), idxUnique(:,1)));
cloud3dFoV_Color = uint8(cloud3dFoV_Color);
cloud3dFoV = double(cloud3dFoV);
end