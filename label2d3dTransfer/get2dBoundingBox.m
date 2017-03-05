function bbox2d = get2dBoundingBox(points)
if size(points, 2) ~= 2
    points = points';
end
maxXY = [max(points(:, 1)), max(points(:, 2))];
minXY = [min(points(:, 1)), min(points(:, 2))];
bbox2d = [minXY, maxXY - minXY];
end