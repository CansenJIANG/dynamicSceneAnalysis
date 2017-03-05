% function to plot the point cloud trajactory of size(3FxP)
% function plot3dTrajectories(Cloud3d, colorIdx)
function plot3dTrajectories(Cloud3d, colorIdx)

rows = size(Cloud3d,1);
cols = size(Cloud3d,2);

if length(colorIdx) == 1
    plot3(Cloud3d(1,:), Cloud3d(2,:), ...
              Cloud3d(3,:), 'b-');
    for i = 1:cols
        point_i = reshape(Cloud3d(:,i),[3, rows/3]);
        plot3(point_i(1,:), point_i(2,:), point_i(3,:), colorIdx);
    end
else
    % Set color type
    for c = 1:length(unique(colorIdx))
        if c == 1
            color(1) = 'r';
        elseif c == 2
            color(2) = 'g';
        elseif c == 3
            color(3) = 'b';
        elseif c == 4
            color(4) = 'm';
        elseif c == 5
            color(5) = 'c';
        elseif c == 6
            color(6) = 'k';
        else
            color(c) = color(randi(6));
        end
    end

    % Plot trajectories
    for i = 1:length(colorIdx)    
        point_i = reshape(Cloud3d(:,i),[3,rows/3]);
        plot3dTrajectory(point_i',['-', color(colorIdx(i))]);
    end
end
end


