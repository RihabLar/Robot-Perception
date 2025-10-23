function [avg_proj_error] = average_projection_error(pts_2d, pts_2d_noisy)
% Compute average projection error
n = length(pts_2d);
all_dists = 0;
for i = 1:n
    dist = sqrt((pts_2d(i, 1) - pts_2d_noisy(i, 1))^2 + ((pts_2d(i, 2) - pts_2d_noisy(i, 2))^2));
    all_dists = all_dists + dist;
end
avg_proj_error = all_dists / n;
end

