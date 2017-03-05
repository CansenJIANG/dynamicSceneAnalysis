%% Estimate object velocity
function [objectList] = updateObjectListMotionInfo(objectList, noGndRegistT0, noGndRegistT1,...
    cloudRegistT0, tform, Tmapping, P_velo_to_img, leftImg)
for objIdx = 1:length(objectList)
    objMap = objectList{objIdx};
    if objMap.speed < 0.5
        if objMap.FoV == 0
            continue;
        end
        objSegMap = objMap.objSegMap;
        objSegMap = (tform.T')*pinv(Tmapping)*[objSegMap'; ones(1,size(objSegMap, 1))];
        [bbCloud_img] = project(objSegMap(1:3,:)', P_velo_to_img);
        objMap.bb2d = get2dBoundingBoxImage(bbCloud_img);
        objectList{objIdx} = objMap;
        continue;
    end
    
    noGndRegistT0_ = tform.T'*pinv(Tmapping)*[noGndRegistT0'; ones(1, size(noGndRegistT0, 1))];
    noGndRegistT0_ = noGndRegistT0_(1:3, :)';
    %     if ~isempty(objMap.trkObj)
    %         [tOptimal, angularVelocity, linearVelocity, trkObj, bboxMapUpdate, bbAxisUpdate] = ...
    %             locateObjectMotion(noGndRegistT0, noGndRegistT1, noGndRegistT0_, ...
    %             objMap.bb3dMap, objMap.bbAxis, objMap.label,  objMap.trkObj);
    %     else
    [tOptimal, angularVelocity, linearVelocity, trkObj, bboxMapUpdate, bbAxisUpdate] =...
        estimateObjectMotion(noGndRegistT0,...
        noGndRegistT1, noGndRegistT0_, objMap.bb3dMap, objMap.bbAxis, objMap.label);
    %     end
    
    objMap.trkObj = trkObj;
    objMap.speed = linearVelocity;
    if linearVelocity > 0.5
        offsetMax = 0.4; offsetMin = 0;
        % [maxX, minX, maxY, minY, maxZ, minZ]
        offsetXYZ = [0., 0., 0., 0., 0.4, 0];
        objSegMap = segmentCloudBbox3d(cloudRegistT0(1:3,:)', objMap.bb3dMap, ...
            offsetXYZ);
    else
        offsetXYZ = [0, -0, 0, -0, 0.0, -0.250];
        objSegMap = segmentCloudBbox3d(cloudRegistT0(1:3,:)', objMap.bb3dMap, offsetXYZ);
    end
    objSegMap = (tform.T')*pinv(Tmapping)*[objSegMap'; ones(1,size(objSegMap, 1))];
    if 0
        figure(20), showPointCloud(objSegMap, [1 0 0]); hold on; 
        showPointCloud(cloudRegistT0(1:3, :)');plotCube(objMap.bb3dMap, [0 0 1]);
    end
    [bbCloud_img] = project(objSegMap(1:3,:)', P_velo_to_img);
    objMap.bb2d = get2dBoundingBoxImage(bbCloud_img);
    objMap.bb3dMap = bboxMapUpdate;
    objectList{objIdx} = objMap;
    if 0
        figure(12); imshow(leftImg);
        rectangle('Position', objMap.bb2d, 'EdgeColor', [1 0 0], 'LineWidth', 3);
        drawnow;
    end
    %
    %     if(norm(objMap.bboxMap-bboxMapUpdate)<0.5) || isempty(tOptimal)
    %         objectList{objIdx}.state = 0;
    %         figure(1001); hold on;
    %         str = sprintf('ID:%02d %s',objectList{objIdx}.ObjNb,...
    %             objectList{objIdx}.label);
    %         H= text(objectList{objIdx}.bbox2d(1) -20,...
    %             objectList{objIdx}.bbox2d(2) - 20, str);
    %         H.BackgroundColor = [0 0 0];
    %         H.FontSize = 15; hold off;
    %
    %     else
    %         % draw object information
    %         if cIdx == 1
    %             figure(1001); hold on;
    %             str = sprintf('ID:%02d %s\nSpeed: %.01f m/s',objectList{objIdx}.ObjNb,...
    %                 objectList{objIdx}.label, linearVelocity);
    %             H= text(objectList{objIdx}.bbox2d(1)+objectList{objIdx}.bbox2d(3)+10,...
    %                 objectList{objIdx}.bbox2d(2)+0.5*objectList{objIdx}.bbox2d(4), str);
    %             H.BackgroundColor = [0 0 0];
    %             H.FontSize = 15; hold off;
    %             H.Color = [0 1 0];
    %         end
    %         [velo3dRegMap, velo3dRegFoV_Color]= removeMovingObjects(velo3dRegMap(1:3,:)', velo3dRegFoV_Color, objMap.bboxMap);
    %         centerTraj = [mean(objMap.bboxMap); mean(bboxMapUpdate)];
    %         objMap.bboxMap = bboxMapUpdate;
    %         objMap.bbAxis = bbAxisUpdate;
    %         colorDraw = getDrawColor(colors, objMap.label);
    %         objMap.centerTraj = [objMap.centerTraj; centerTraj];
    %         objMap.speed = linearVelocity;
    %         objectList{objIdx} = objMap;
    %         %             figure(1000), hold on; plot3(centerTraj(:,1),centerTraj(:,2),centerTraj(:,3),'Color', colorDraw, 'LineWidth', 3);
    %         %                         plotCube(objectTmp.bboxMap, colorDraw);
    %         hold off;
    %         if cIdx == params.frmEnd
    %             %                 figure(1000), hold on; plotCube(objectTmp.bboxMap, colorDraw);
    %         end
    %     end
end
end