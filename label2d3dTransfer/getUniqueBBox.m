%% Function detect the intersections on both 2D and 3D bounding box of objects
%% to combine multi-detected same objects.
function bbCandidate = getUniqueBBox(cloudNoGndFoV, cloudNoGndProj3DUnique,...
    cloudFull, P_velo_to_img, Tmapping, tform, ...
    bbox, objLabels, objProb, bbNb, frmIdx)
%% candidate Object Bounding Boxes
bbCandidate = [];
for ibox = 1:bbNb
    objIdx_x = intersect(find(cloudNoGndProj3DUnique(1, :)>bbox(ibox, 1)),...
        find(cloudNoGndProj3DUnique(1, :)<bbox(ibox, 3)));
    objIdx_y = intersect(find(cloudNoGndProj3DUnique(2, :)>bbox(ibox, 2)),...
        find(cloudNoGndProj3DUnique(2, :)<bbox(ibox, 4)));
    objIdx = intersect(objIdx_x, objIdx_y);
    if length(objIdx) < 10
        continue;
    end
    boxLabel = objLabels{ibox};
    bbCloud = double(cloudNoGndFoV(objIdx,:));
    [obj] = removeOutlierObjects(bbCloud, objLabels{ibox});
    [cloudSeg, idx, bboxSeg, bbAxis] = regionGrow3dPointCloud(cloudNoGndFoV, obj, boxLabel);
    % Update bboxSeg using full 3d cloud
    [bbCloud_img, objCloud] = preciseObjectLocalization(cloudFull, bboxSeg, P_velo_to_img, bbAxis);
    [bb2d] = get2dBoundingBoxImage(bbCloud_img);
    if 0
        figure(1), showPointCloud(cloudSeg, [1 0 0]); hold on; showPointCloud(objCloud, [0 0 1]);
        figure(01); clf; showPointCloud(obj, [1 0 0], 'MarkerSize', 20);
        hold on; showPointCloud(cloudSeg, [0 0 1], 'MarkerSize', 10);
    end
    % Get object bounding box volume
    [bboxVol, bbX, bbY, bbZ] = get3dBBoxVolumn(bboxSeg, bbAxis);
    if (strcmp(boxLabel, 'car') && max([bbX, bbY, bbZ]) > 4.0 ) ||...
       (strcmp(boxLabel, 'bicycle') && max([bbX, bbY, bbZ]) > 2.5 ) ||...
       (strcmp(boxLabel, 'person') && max([bbX, bbY, bbZ]) > 2.0 ) ||...
       (strcmp(boxLabel, 'truck') && bboxVol < 10)
        continue;
    end
    % Get 3d bounding box in Map location
    bboxMap = Tmapping*pinv(tform.T')*[bboxSeg'; ones(1, 8)]; % bounding box in map frame
    bboxMap = bboxMap(1:3, :)';
    cloudSegMap = Tmapping*pinv(tform.T')*[cloudSeg';ones(1, size(cloudSeg, 1))];
    cloudSegMap = cloudSegMap(1:3, :)';
    
    % Save object information
    objParam.bb3dCam = bboxSeg;
    objParam.bb2d = bb2d;
    objParam.label = boxLabel;
    objParam.objSeg = cloudSeg;
    objParam.objSegMap = cloudSegMap;
    objParam.bbAxis = bbAxis;
    objParam.prob = objProb(ibox);
    objParam.size = bboxVol;
    objParam.speed = 1;
    objParam.bb3dMap = bboxMap;
    objParam.center = mean(bboxMap);
    objParam.frmIdx = frmIdx;
    objParam.labelist{1} = boxLabel;
    bbCandidate{end+1} = objParam;
    objParam = [];
end
%% Candidate Connected Bounding Boxes
cnCandidate = []; % connected bounding box candidates
for ibox = 1:length(bbCandidate)
    for jbox = ibox+1:length(bbCandidate)
        bbox1 = bbCandidate{ibox}.bb3dCam;
        bbox2 = bbCandidate{jbox}.bb3dCam;
        label1 = bbCandidate{ibox}.label;
        label2 = bbCandidate{jbox}.label;
        bb2d_1 = bbCandidate{ibox}.bb2d;
        bb2d_2 = bbCandidate{jbox}.bb2d;
        area = rectint(bb2d_1,bb2d_2);
        if ~area
            continue;
        end
        bbConnect = [];
        intersect3d = get3dBboxIntersection(bbox1, bbox2);
        if intersect3d > 0
            if  strcmp(label1, label2)
                bbConnect.label = label1;
                bbConnect.bb2d = [0.5*(bb2d_1(1:2)+bb2d_2(1:2)),...
                max(bb2d_1(3), bb2d_2(3)), max(bb2d_1(4), bb2d_2(4))];
                bbConnect.prob = [bbCandidate{ibox}.prob, bbCandidate{jbox}.prob];
                
            elseif (strcmp(label1, 'person') && strcmp(label2, 'bicycle') ||...
                    strcmp(label1, 'bicycle') && strcmp(label2, 'person') )
                bbConnect.label = 'cyclist';
                bbConnect.bb2d = [min(bb2d_1(1), bb2d_2(1)), min(bb2d_1(2), bb2d_2(2)),...
                max(bb2d_1(1)+bb2d_1(3), bb2d_2(1)+bb2d_2(3)),...
                min(bb2d_1(2)+bb2d_1(4), bb2d_2(2)+bb2d_2(4))];
                bbConnect.bb2d(3:4) = bbConnect.bb2d(3:4) - bbConnect.bb2d(1:2);
                bbConnect.prob = [bbCandidate{ibox}.prob, bbCandidate{jbox}.prob];
            else
                if bbCandidate{ibox}.prob >= bbCandidate{jbox}.prob
                    bbConnect.label = label1;
                else
                    bbConnect.label = label2;
                end
                bbConnect.bb2d = [0.5*(bb2d_1(1:2)+bb2d_2(1:2)),...
                max(bb2d_1(3), bb2d_2(3)), max(bb2d_1(4), bb2d_2(4))];
                bbConnect.prob = [bbCandidate{ibox}.prob, bbCandidate{jbox}.prob];
            end
            bbConnect.bbIdx = [ibox, jbox];
            cnCandidate{end+1} = bbConnect;
        end
    end
end

%% Update Object Bounding Boxes Candidates
rmBBIdx = [];
for iCn = 1:length(cnCandidate)
    % Get precise 3d object segmentation
    bbConnect = cnCandidate{iCn};
    bboxCn = [bbConnect.bb2d(1:2), bbConnect.bb2d(1:2) + bbConnect.bb2d(3:4)];
    objIdx_x = intersect(find(cloudNoGndProj3DUnique(1, :)>bboxCn(1)),...
        find(cloudNoGndProj3DUnique(1, :)<bboxCn( 3)));
    objIdx_y = intersect(find(cloudNoGndProj3DUnique(2, :)>bboxCn(2)),...
        find(cloudNoGndProj3DUnique(2, :)<bboxCn(4)));
    objIdx = intersect(objIdx_x, objIdx_y);
    if isempty(objIdx)
        continue;
    end
    boxLabel = bbConnect.label;
    bbCloud = double(cloudNoGndFoV(objIdx,:));
    [obj] = removeOutlierObjects(bbCloud, boxLabel);
    [cloudSeg, idx, bboxSeg, bbAxis] = regionGrow3dPointCloud(cloudNoGndFoV, obj, boxLabel);
    
    % Get object bounding box volume
    [bboxVol, bbX, bbY, bbZ] = get3dBBoxVolumn(bboxSeg, bbAxis);
    
    % Get 3d bounding box in Map location
    bboxMap = Tmapping*pinv(tform.T')*[bboxSeg'; ones(1, 8)]; % bounding box in map frame
    bboxMap = bboxMap(1:3, :)';
    cloudSegMap = Tmapping*pinv(tform.T')*[cloudSeg';ones(1, size(cloudSeg, 1))];
    cloudSegMap = cloudSegMap(1:3, :)';

    if 0
        figure(01); clf; showPointCloud(obj, [1 0 0], 'MarkerSize', 20);
        hold on; showPointCloud(cloudSeg, [0 0 1], 'MarkerSize', 10);
    end
    % Save object information
    objParam.bb3dCam = bboxSeg;
    objParam.bb2d = bbConnect.bb2d;
    objParam.label = boxLabel;
    objParam.objSeg = cloudSeg;
    objParam.objSegMap = cloudSegMap;
    objParam.bbAxis = bbAxis;
    objParam.prob = 0.8*sum(bbConnect.prob);
    objParam.size = bboxVol;
    objParam.speed = 1;
    objParam.bb3dMap = bboxMap;
    objParam.center = mean(bboxMap);
    objParam.frmIdx = frmIdx;
    objParam.labelist{1} = boxLabel;
    bbCandidate{end+1} = objParam;
    rmBBIdx = [rmBBIdx, bbConnect.bbIdx];
    objParam = [];
end
rmBBIdx = unique(rmBBIdx);
for j = 1:length(rmBBIdx)
    bbCandidate{1, rmBBIdx(j) } = [];
end
bbCandidate = bbCandidate(~cellfun(@isempty, bbCandidate));
end