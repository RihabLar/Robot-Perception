function [pts_2d_noisy] = add_Gaussian_noise(pts_2d)
% Add Gaussian noise to 2D points
[m, n] = size(pts_2d);
pts_2d_noisy = zeros(m, n);
for i = 1:m
 x_noise = normrnd(0, 1/1.96);
 y_noise = normrnd(0, 1/1.96);
 pts_2d_noisy(i, 1) = pts_2d(i, 1) + x_noise;
 pts_2d_noisy(i, 2) = pts_2d(i, 2) + y_noise;
end
end
