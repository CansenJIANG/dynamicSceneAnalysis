function [objList] = drawObjectList(objList, colors, img)
for i=1:length(objList)
    objMap = objList{i};
    if objMap.FoV == 0
        continue;
    end
    [FoV] = drawBBox(objMap.bb2d, objMap.bb3dMap, objMap.label, objMap.ID, objMap.speed, colors, img);
    %     drawLabel(objMap.bb2d, objMap.label, objMap.ID, objMap.speed);
    if FoV==0
        objMap.FoV = 0;
        objList{i} = objMap;
        display('object disappeared');
    end
end
end

function FoV = drawBBox(bb2d, bb3dMap, label, objID, speed, colors, img)
colorDraw = getDrawColor(label, colors);
[FoV, bb2d] = checkFoV(bb2d, img);
if FoV == 0
    return;
end
figure(1001); hold on;
if sum(find(bb2d<0))
    FoV = 0;
    return;
end
rectangle('Position', bb2d, 'EdgeColor', colorDraw, 'LineWidth', 3);
drawnow; drawLabel(bb2d, label, objID, speed);
hold off;
% figure(1000); hold on; plotCube(bb3dMap, colorDraw); drawnow;  hold off;
end

function drawLabel(bb2d, label, objID, speed)
if strcmp(label, 'traffic_light')
    label = 'traffic light';
end
if speed < 0.5
    figure(1001); hold on;
    str = sprintf('ID:%02d %s', objID, label);
    H= text(bb2d(1)-18, bb2d(2)-18, str);
    H.BackgroundColor = [0 0 0];
    H.Color = [1 0 0];
    H.FontSize = 15;
    drawnow;
    hold off;
else
    figure(1001); hold on;
    str = sprintf('ID:%02d %s\nSpeed: %.01f m/s', objID, label, speed);
    H= text(bb2d(1)+bb2d(3)+5,...
        bb2d(2)+0.5*bb2d(4), str);
    H.BackgroundColor = [0 0 0];
    H.FontSize = 15;
    H.Color = [0 1 0];
    drawnow; hold off;
end
end

function [FoV, bb2d]= checkFoV(bb2d, img)
[cols, rows] = size(img(:, :, 1));
if isempty(bb2d)
    FoV = 0;
    return;
end
    
bb2d_ = bb2d;
bb2d_(3:4) = bb2d_(1:2) + bb2d_(3:4);
if bb2d(1) > rows || bb2d(2) > cols  ||...
        bb2d(3) > rows || bb2d(4) > cols ||...
        bb2d(3) < 0 || bb2d(4) < 0
    FoV = 0;
    display('object disappeared');
    return;
end
if bb2d_(3) < 0 || bb2d_(4) < 0
    FoV = 0;
    display('object disappeared');
    return;
end
bb2d_(1) = max(1, bb2d_(1));
bb2d_(2) = max(1, bb2d_(2));
bb2d_(3) = min(rows, bb2d_(3));
bb2d_(4) = min(cols, bb2d_(4));
bb2d = [bb2d_(1:2), bb2d_(3:4) - bb2d_(1:2)];
FoV = 1;
end