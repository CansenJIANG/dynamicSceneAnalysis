% function to plot the point cloud trajactory of size(2FxP)
% function plot2dTrajectories(Cloud2d, colorIdx)
function plot2dTrajectories(Cloud2d, colorIdx)

rows = size(Cloud2d,1);
cols = size(Cloud2d,2);

if length(colorIdx) == 1
    plot(Cloud2d(1,:), Cloud2d(2,:), 'b*');
    for i = 1:cols
        point_i = reshape(Cloud2d(:,i),[2, rows/2]);
        plot(point_i(1,:), point_i(2,:), '-sr');
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
        else
            color(6) = 'k';
        end
    end

    % Plot trajectories
    for i = 1:length(colorIdx)    
        point_i = reshape(Cloud2d(:,i),[2,rows/2]);
        plot2dTrajectory(point_i',color(colorIdx(i)));
    end
end
end


