function [pts_2d] = convert_to_2d(pts_3d_proj)
% Convert to 2D
n = length(pts_3d_proj);
pts_2d = [];
for i = 1:n
    pt_proj = pts_3d_proj(i, :)';
    pt_2d = pt_proj(1:2) ./ pt_proj(3);
    pts_2d = [pts_2d; pt_2d'];
end
end
