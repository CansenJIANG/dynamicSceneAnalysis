figure(4000), showPointCloud(fullMap, uint8(fullMapColor));
for i = 1:length(objectList)
     figure(4000),hold on; 
     objectTmp = objectList{i};
     colorDraw = getDrawColor(colors, objectTmp.label);
     if objectTmp.state
        centerTraj = objectTmp.centerTraj;
        if ~isempty(centerTraj)
            plot3(centerTraj(:,1),centerTraj(:,2),centerTraj(:,3),'Color', colorDraw, 'LineWidth', 3);
        end
     end
     plotCube(objectTmp.bboxMap, colorDraw);
end