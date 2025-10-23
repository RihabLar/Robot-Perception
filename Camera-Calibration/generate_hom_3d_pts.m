function [pts_hom_3d] = generate_hom_3d_pts(n, lower, upper)
% Randomly generate a set of n homogeneous 3d points within the range (lower, upper)
pts_cart_3d = randi([lower, upper], n, 3);
pts_hom_3d = [pts_cart_3d, ones(n, 1)];
end
