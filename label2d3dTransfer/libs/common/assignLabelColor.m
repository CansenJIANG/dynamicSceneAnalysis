%% Function assign most distinctive colors from color labels
%% Input: labels of class (or color)
%% Output: color with same length of labels
function [lblC] = assignLabelColor(lbl)
maxLbl = max(lbl);
clrOpt = distinguishable_colors(maxLbl);
lblC = zeros(length(lbl), 3);
for i=1:maxLbl
    lblC(lbl == i, :) = repmat(clrOpt(i, :), [sum(lbl == i),1]);
end
end