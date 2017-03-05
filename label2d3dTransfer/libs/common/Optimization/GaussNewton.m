% optimization using Gauss-Newton
% % % 


clear;clc;close all;
syms x y
f(x, y) = 100*(y-x*x)^2 + (1-x)^2;
% x = linspace(-4,1); y = linspace(-1,5);
% [xx,yy] = meshgrid(x,y); ff = f(xx,yy);
% levels = 1:10:300;
% LW = 'linewidth'; FS = 'fontsize'; MS = 'markersize';
% figure, contour(x,y, ff, levels, LW,1.2), colorbar
% axis([-4 1 -1 5]), axis square, hold on

%% Gradient Descent
grad(x, y) = [100*(-4*x*y+4*x^3)+2*x-2; 100*(2*y-2*x^2)];
%% Hessian matrix
H(x,y)=[-400*(y-3*x^2)+2, -400*x; -400*x, 200];
%% Jacobian matrix
J(x,y) = [100*(-4*y+4*x^3)+2*x-2, 100*(2*y-2*x^2)]; % Jacobian

x = -1.9; y = 2.0; step = 1; gradthresh = 0.000001;
f0 = vpa(f(x, y));
for i = 1:100
    
%% Steepest descent direction
% dxdy = vpa(grad(x, y)*step);

%% Newton descent
dxdy = vpa(-inv(H(x, y))*grad(x, y)*step);
x = x + dxdy(1); y = y + dxdy(2);
% display( vpa(f(x, y)));
if( abs(f0 - vpa(f(x, y))) < gradthresh) 
    break;
else
    f0 = vpa(f(x, y));
end
end
display(['Newton descent residual is: ', num2str(double(f(x, y))), 'with ', num2str(i), 'iteration']);

%% Quasi-Newton
x = -1.9; y = 2.0; step =1; gradthresh = 0.000001;
H_QN = H(x,y);
f0 = vpa(f(x, y));
for i = 1:200
xn = x; yn = y;
gn = grad(x, y);
dxdy = vpa(-inv(H_QN)*grad(x, y)*step);
x = x + dxdy(1); y = y + dxdy(2);
% display( vpa(f(x, y)));
if( abs(f0 - vpa(f(x, y))) < gradthresh) 
    break;
else
    f0 = vpa(f(x, y));   
end
sn = [x - xn; y - yn];
qn = [grad(x, y) - gn];
H_QN = H_QN + qn*qn'/(qn'*sn) - (H_QN*sn)*(H_QN*sn)'/(sn'*H_QN*sn);
end
display(['Quasi-Newton descent residual is: ', num2str(double(f(x, y))), 'with ', num2str(i), 'iteration']);

%% Gauss-Newton
x = -1.9; y = 2.0; step = 1; gradthresh = 0.000001;
f0 = vpa(f(x, y));
for i = 1:100
%% Hessian matrix approximation using Jacobian
H_hat = 2*J(x,y)'*J(x,y);
%% Newton descent
dxdy = vpa(-pinv(H_hat)*grad(x, y)*step);
x = x + dxdy(1); y = y + dxdy(2);
% display( vpa(f(x, y)));
if( abs(f0 - vpa(f(x, y))) < gradthresh) 
    break;
else
    f0 = vpa(f(x, y));
end
end
display(['Gauss-Newton residual is: ', num2str(double(f(x, y))), 'with ', num2str(i), 'iteration']);

