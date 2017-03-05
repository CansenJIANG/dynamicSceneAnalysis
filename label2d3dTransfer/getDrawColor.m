function colorDraw = getDrawColor(objLabels, colors)
if nargin < 2
    colors = distinguishable_colors(7);
end

if strcmp(objLabels, 'car')
    colorDraw = colors(3,:);
elseif strcmp(objLabels, 'person')% || strcmp(objLabels, 'cyclist')
    colorDraw = colors(2,:);
elseif strcmp(objLabels, 'bicycle')
    colorDraw = colors(1,:);
elseif strcmp(objLabels, 'motorbike')
    colorDraw = colors(4,:);
elseif strcmp(objLabels, 'cyclist')
    colorDraw = colors(5,:);
elseif strcmp(objLabels, 'traffic_light')
    colorDraw = colors(6,:);
else
    colorDraw = colors(7,:);
end
end