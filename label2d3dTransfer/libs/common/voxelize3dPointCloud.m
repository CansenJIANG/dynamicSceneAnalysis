%% filter 3d point cloud to using 3D voxel
%% opt: 'gridSize' or 'gridNumber'
function [cntEmpty,voxelOccupancy] = voxelize3dPointCloud(pc3, opt, resol)
if size(pc3,1)<4
    pc3 = pc3';
end

maXYZ = max(pc3);
miXYZ = min(pc3);

if strcmp(opt,'axisGridNumber')
    resol_ = (max(maXYZ-miXYZ))/resol*ones([3,1]);
    
    [x, y, z] = meshgrid(miXYZ(1):resol_(1):maXYZ(1),...
        miXYZ(2):resol_(2):maXYZ(2),...
        miXYZ(3):resol_(3):maXYZ(3));
    gridCens = [x(:),y(:),z(:)];
else strcmp(opt,'totalGridNumber')
    resol_ = (maXYZ-miXYZ)/resol;
    x = miXYZ(1):resol_(1):maXYZ(1);
    y = miXYZ(2):resol_(2):maXYZ(2);
    z = miXYZ(3):resol_(3):maXYZ(3);
end


% construct kdtree for fast searching
if resol_(1)~=resol_(2)
    values = 1:resol;
    x_ = discretize(pc3(:,1),x,values);
    y_ = discretize(pc3(:,2),y,values);
    z_ = discretize(pc3(:,3),z,values);
    
    xyz_ = [x_, y_,z_];
    gridsNb = resol*resol*resol;
    cntEmpty = gridsNb - length(unique(xyz_,'rows'));
    if(length(pc3)<gridsNb)
        cntEmpty = cntEmpty*length(pc3)/gridsNb;
    end
%     else
        voxelOccupancy = cntEmpty/gridsNb;
%     end
        
%     sparsityIdx = cntEmpty*length(pc3)/gridsNb;
else
    MdlKDT = createns(pc3,'NSMethod','kdtree','Distance','cityblock');
    idx = rangesearch(MdlKDT,gridCens,resol_(1),'Distance','cityblock');
    
    cntEmpty = 0;
    for i=1:length(idx)
        if isempty(idx{i})
            cntEmpty = cntEmpty + 1;
        end
    end
    voxelOccupancy = cntEmpty/length(gridCens);
end

end
