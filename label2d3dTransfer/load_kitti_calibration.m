function P_velo_to_img = load_kitti_calibration(calibFile, useKittiObject)
    % Get the relative pose between 3D camera and 2d image
    fid = fopen(fullfile(calibFile),'r');
    P2  = readVariable(fid,'P2',3,4);
    P3  = readVariable(fid,'P3',3,4);
    if useKittiObject
        Tr  = readVariable(fid,'Tr_velo_cam',3,4);
    else
        Tr  = readVariable(fid,'Tr',3,4);
    end
    R_rect =  readVariable(fid,'R_rect',3,3);
    Imu2Velo = readVariable(fid,'Tr_imu_velo',3,4);
    fclose(fid);
    if isempty(R_rect)
        R_rect = eye(4,4);
        Imu2Velo = eye(4,4);
    end
    % Get intrinsic parameters
    K2 = P2(1:3, 1:3);
    K3 = P3(1:3, 1:3);
    
    % To project a point from Velodyne coordinates into the left color image,
    % you can use this formula: x = P2 * R0_rect * Tr_velo_to_cam * y
    % For the right color image: x = P3 * R0_rect * Tr_velo_to_cam * y
    R_rect(4,4) = 1;
    P_velo_to_img = P2*R_rect*[Tr; 0, 0, 0, 1];
%     paramCfg.P_velo_to_img = P_velo_to_img;
end