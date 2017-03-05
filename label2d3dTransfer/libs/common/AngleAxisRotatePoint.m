function result = AngleAxisRotatePoint(angle_axis, pts)
theta2 = dot(angle_axis, angle_axis);
%    Away from zero, use the rodriguez formula
%
%    result = pt costheta +
%             (w x pt) * sintheta +
%             w (w . pt) (1 - costheta)
%
%     We want to be careful to only evaluate the square root if the
%     norm of the angle_axis vector is greater than zero. Otherwise
%     we get a division by zero.

if(size(pts, 1)~=3)
    pts = pts';
end

theta = sqrt(theta2);
costheta = cos(theta);
sintheta = sin(theta);
theta_inverse = 1.0 / theta;

w = [ angle_axis(1) * theta_inverse;
    angle_axis(2) * theta_inverse;
    angle_axis(3) * theta_inverse ];

%  Explicitly inlined evaluation of the cross product for
%  performance reasons.
% w_cross_pt = [ w(2) * pt(3) - w(3) * pt(2);
%     w(3) * pt(1) - w(1) * pt(3);
%     w(1) * pt(2) - w(2) * pt(1) ];

w_cross_pt = [ w(2) .* pts(3, :) - w(3) .* pts(2, :);
               w(3) .* pts(1, :) - w(1) .* pts(3, :);
               w(1) .* pts(2, :) - w(2) .* pts(1, :)];


% tmp = (w(1) * pt(1) + w(2) * pt(2) + w(3) * pt(3)) * (1.0 - costheta);
% result = zeros(3, 1);
% result(1) = pt(1) * costheta + w_cross_pt(1) * sintheta + w(1) * tmp;
% result(2) = pt(2) * costheta + w_cross_pt(2) * sintheta + w(2) * tmp;
% result(3) = pt(3) * costheta + w_cross_pt(3) * sintheta + w(3) * tmp;

tmp = (w(1) * pts(1, :) + w(2) * pts(2, :) + w(3) * pts(3, :)) * (1.0 - costheta);
result(1, :) = costheta .* pts(1, :) + sintheta .* w_cross_pt(1, :) + w(1) .* tmp;
result(2, :) = costheta .* pts(2, :) + sintheta .* w_cross_pt(2, :) + w(2) .* tmp;
result(3, :) = costheta .* pts(3, :) + sintheta .* w_cross_pt(3, :) + w(3) .* tmp;
end
