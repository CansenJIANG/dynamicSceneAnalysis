% function reorder the matrix according to the group size
% matNew = reorderMatSeq(m, sz, orderDirec)
% sz: group size
% orderDirec: 'down2up' or 'right2left'
function matNew = reorderMatSeq(m, sz, orderDirec)
if nargin<3
    orderDirec = 'down2up';
end

if nargin<2
    sz = 2;
end
matNew = [];
switch orderDirec
    case 'down2up'
        for i=1:size(m, 1)/sz
            matNew = [matNew; m(end-i*sz+1:end-(i-1)*sz,:)];
        end
    case 'right2left'
        for i=1:size(m, 2)/sz
            matNew = [matNew, m(:, end-i*sz+1:end-(i-1)*sz)];
        end
end

end