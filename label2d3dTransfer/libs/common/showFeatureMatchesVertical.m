%[] = showFeatureMatchesVertical(I1, I2, Feats1, Feats2)
function [] = showFeatureMatchesVertical(I1, I2, Feats1, Feats2)
I = [I1; I2];
if size(Feats1, 1)== 2; Feats1 = Feats1'; end
if size(Feats2, 1)== 2; Feats2 = Feats2'; end
x1 = Feats1(:,1); y1 = Feats1(:,2);
x2 = Feats2(:,1); y2 = Feats2(:,2) + size(I1,1)*ones(length(Feats2), 1);

if(any(findall(0,'Type','Figure')==999999))
    figure;
else
    figure(999999);
end
imshow(I), hold on; 
plot(x1, y1, '+r', 'MarkerSize', 10,'LineWidth', 3);
plot(x2, y2, 'ob', 'MarkerSize', 10,'LineWidth', 3);

for i = 1:length(x1)
    plot([x1(i); x2(i)], [y1(i), y2(i)], '-g');    
end
end