%% Function computer the intersection between new object with existing objects
%% that has been tracked in the 3D map.
%% Also update object information if necessary
function [objectList] = updateObjectList(objectList, bbCandidate)
for iobjCan = 1:length(bbCandidate)
    objCan = bbCandidate{iobjCan};
    addNew = 1;
    for iobjLst = 1:length(objectList)
        objMap = objectList{iobjLst};
        if ~objMap.FoV
            continue;
        end
        volInter3d = get3dBboxIntersection(objMap.bb3dMap, objCan.bb3dMap);
        if volInter3d>0 
            boxVol1 = get3dBBoxVolumn(objMap.bb3dMap);
            boxVol2 = get3dBBoxVolumn(objCan.bb3dMap);
            if( norm(mean(objMap.bb3dMap) - mean(objCan.bb3dMap)) > 1.5 &&...
                    (volInter3d / max(boxVol1, boxVol2) < 0.1) )
                continue;
            end            
                
            % Intersection occur
            %             if strcmp(objMap.label, objCan.label)
            objMap.prob  = [objMap.prob, objCan.prob];
            orgLabel = objMap.label;
            objMap.labelist{end+1} = orgLabel;
            if length(objMap.prob) >= 10 && objMap.speed > 0.5
                objMap.label = orgLabel;
            elseif length(objMap.prob) >= 10
                objMap.label = getMajorVoting(objMap.labelist);
%                 objMap.label = objMap.labelist{find(objMap.prob==max(objMap.prob))};
            else
                objMap.label = getMaxpooling(objMap.prob, objMap.labelist);
            end
            if ~strcmp(orgLabel, objMap.label)
                objMap.bb3dMap = objCan.bb3dMap;
                objMap.objSegMap = objCan.objSegMap;
                objMap.objSeg = objCan.objSeg;
            end
            objMap.labelist{end} = objMap.label;
            objMap.bb2d = objCan.bb2d;
%             objMap.objSegMap = objCan.objSegMap;
            objectList{iobjLst} = objMap;
            addNew = 0;
            break;
            if 0
                figure(1), showPointCloud([0 0 0]); hold on;
                plotCube(objMap.bb3dMap, [1 0 0]);
                plotCube(objCan.bb3dMap, [0 0 1]);
            end
        end
    end
    if addNew
        objCan.ID = length(objectList) + 1;
        objCan.FoV = 1; % Object can be seen by 2D camera
        objCan.trkObj = [];  % moving object in next frame
        objectList{end+1} = objCan;
    end
end


end

function label = getMajorVoting(labelist)
ulabelist = unique(labelist);
n = zeros(length(ulabelist), 1);
for iy = 1:length(ulabelist)
  n(iy) = length(find(strcmp(ulabelist{iy}, labelist)));
end
[~, itemp] = max(n);
label = ulabelist{itemp};
end


function label = getMaxpooling(prob, labelist)
label = labelist{find(prob==max(prob))};
end