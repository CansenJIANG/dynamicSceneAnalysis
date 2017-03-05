function points_ro_tr = rigidTransformRodrigues(points, Ro, Tr)

angle_axis = rodrigues(Ro);

points_ro = AngleAxisRotatePoint(angle_axis, points);

points_ro_tr(1, :) = points_ro(1, :) + Tr(1);
points_ro_tr(2, :) = points_ro(2, :) + Tr(2);
points_ro_tr(3, :) = points_ro(3, :) + Tr(3);
end
