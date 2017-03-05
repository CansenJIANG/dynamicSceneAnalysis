% function to plot the 3d point trajectory
% cube: size Nx3
% color: choose plotting color, default blue
% function []=plot3dTrajectory(cube, color)
function []=plot3dProjMatching(im1, im2, p1,p2,matchIdx)
if size(p1,1)==3
    p1 = p1';
end
if size(p2,1)==3
    p2 = p2';
end
figure(2000); clf; hold on; view(3);
for i= 1:length(p1)
    X = [p1(i,1);p2(matchIdx(i),1)];
    Y = [p1(i,2);p2(matchIdx(i),2)];
    plot(p1(i,1),p1(i,2),'sr','Markersize', 8);
    plot(p2(matchIdx(i),1),p2(matchIdx(i),2),'sb','Markersize', 8);
    plot(X,Y,'-','Markersize', 5); 
end
hold off;
%     figure; view(3)
%     X = [p1(:,1);p2(matchIdx,1)];
%     Y = [p1(:,2);p2(matchIdx,2)];
%     Z = [p1(:,2);p2(matchIdx,3)];
%     plot3(X,Y,Z,'-s','Markersize', 5); 
end