function [pts_3d_proj] = project_3d_to_image_plane(pts_3d, P)
% Multiply each 3D point in pts_3d by P
n = length(pts_3d);
pts_3d_proj= [];
for i = 1:n
    pt_proj = P * pts_3d(i, :)';
    pts_3d_proj = [pts_3d_proj; pt_proj'];
end
end

