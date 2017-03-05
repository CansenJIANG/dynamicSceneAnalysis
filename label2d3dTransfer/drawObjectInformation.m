function drawObjectInformation(obj, colors, newbbox2d, speed)
if nargin < 4
    speed = obj.speed;
end
colorDraw = getDrawColor(colors, obj.label);
figure(1001); hold on;
%                 plot(bbCloud_img(:,1),bbCloud_img(:,2),'s','Color', colorDraw);
bbox2d = newbbox2d;
rectangle('Position', bbox2d,...
    'EdgeColor', colorDraw, 'LineWidth', 3);
set(gcf,'DefaultTextColor','red');
if obj.speed < 0.05
    str = sprintf('ID:%02d %s', obj.ObjNb, obj.label);
    H= text(bbox2d(1)-20, bbox2d(2)-20, str);
    H.BackgroundColor = [0 0 0];
    H.FontSize = 15;
else
    str = sprintf('ID:%02d %s\nSpeed: %.01f m/s', obj.ObjNb,...
        obj.label, obj.speed);
    H= text(bbox2d(1)+bbox2d(3)+10,...
        bbox2d(2)+0.5*bbox2d(4), str);
    H.BackgroundColor = [0 0 0];
    H.FontSize = 15; hold off;
    H.Color = [0 1 0];
end

drawnow;
hold off;
end