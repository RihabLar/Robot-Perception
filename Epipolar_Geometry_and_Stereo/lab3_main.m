% This a empty script to help you move faster on you lab work.
clear all;
close all;
clc;

%% Step 1
% Camera 1
au1 = 100; av1 = 120; uo1 = 128; vo1 = 128;
imageSize = [256 256];

%% Step 2
% Camera 2
au2 = 90; av2 = 110; uo2 = 128; vo2 = 128; 
ax = 0.1; by = pi/4; cz = 0.2; % XYZ EULER 
tx = -1000; ty = 190; tz = 230; 

%% Step 3
% Compute intrinsic matrices and projection matrices

K1 = [au1 0 uo1; 0 av1 vo1; 0 0 1]; % Intrisics matrix for camera 1
wR1c = eye(3);   % rotation of camera 1, from the camera to the world coordinate frame
wt1c = [0 0 0]'; % translation of camera 1, from the camera to the world coordinate frame

K2 = [au2 0 uo2; 0 av2 vo2; 0 0 1];  % Intrisics matrix for camera 2
rot_x = [1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
rot_y = [cos(by) 0 sin(by); 0 1 0; -sin(by) 0 cos(by)];
rot_z = [cos(cz) -sin(cz) 0; sin(cz) cos(cz) 0; 0 0 1];
wR2c = rot_x * rot_y * rot_z;   % rotation of camera 2, from camera 2 to camera 1
wt2c = [-1000 190 230]'; % translation of camera 2, from camera 2 to camera 1

% Camera projection matrices P1 and P2
P1 = K1 * [wR1c' -wR1c' * wt1c];
disp('P1:')
disp(P1);

P2 = K2 * [wR2c' -wR2c' * wt2c];
disp('P2:')
disp(P2); 

%% STEP 4
% Analytically obtaining F

wt2c_x = [0 -230 190 ; 230 0 1000; -190 -1000 0 ];
%F =  K-t * [t]x * RKt-1  ; 
F =  inv(K2)' * wR2c' * wt2c_x * inv(K1);
F = F./F(3,3);
% Diplay F
fprintf('Step 4:\n\tAnalytically obtained F:\n');
disp(F);

%% STEP 5
V(:,1) = [100;-400;2000];
V(:,2) = [300;-400;3000];
V(:,3) = [500;-400;4000];
V(:,4) = [700;-400;2000];
V(:,5) = [900;-400;3000];
V(:,6) = [100;-40;4000];
V(:,7) = [300;-40;2000];
V(:,8) = [500;-40;3000];
V(:,9) = [700;-40;4000];
V(:,10) = [900;-40;2000];
V(:,11) = [100;40;3000];
V(:,12) = [300;40;4000];
V(:,13) = [500;40;2000];
V(:,14) = [700;40;3000];
V(:,15) = [900;40;4000];
V(:,16) = [100;400;2000];
V(:,17) = [300;400;3000];
V(:,18) = [500;400;4000];
V(:,19) = [700;400;2000];
V(:,20) = [900;400;3000];

%% STEP 6
% Projection on image planes
cam1_p2d = mvg_projectPointToImagePlane(V,P1);
cam2_p2d = mvg_projectPointToImagePlane(V,P2);

%% STEP 7
% Draw 2D projections on image planes
cam1_fig = mvg_show_projected_points(cam1_p2d(1:2,:),imageSize,'Projected points on image plane 1 (Original)');
cam2_fig = mvg_show_projected_points(cam2_p2d(1:2,:),imageSize,'Projected points on image plane 2 (Original)');

%% STEP 8
% 8 point algorithm to estimate F matrix

% Convert 2D image points to homogeneous coordinates
x1 = [cam1_p2d; ones(1, size(cam1_p2d, 2))];
x2 = [cam2_p2d; ones(1, size(cam2_p2d, 2))];
num_points = size(x1, 2); 

% Construct the matrix A based on the epipolar constraint
% A is an Nx9 matrix, where each row represents a constraint derived from
% Corresponding points in the two views
A = [x1(1,:)'.*x2(1,:)', x1(2,:)'.*x2(1,:)', x2(1,:)', x1(1,:)'.*x2(2,:)', x1(2,:)'.*x2(2,:)', x2(2,:)', x1(1,:)', x1(2,:)', ones(num_points, 1)];

% Get F matrix 
[~, ~, V] = svd(A);
F_8 =reshape(V(:,end), 3, 3)'; % Reshape the last column of V into a 3x3 matrix
% Enforce rank-2 constraint on F
[U, S, V] = svd(F_8);   % Use SVD again on F_8 and set the smallest singular value to zero
S(3, 3) = 0;            % Force the smallest singular value to zero
F_8 = U *S *V';         % Reconstruct the rank-2 F_8
F_8 = F_8 ./ F_8(3, 3); % Normalize F 

% Diplay F
disp('Estimated Fundamental Matrix (F_8): ');
disp(F_8);

%% Step 9
% Compute the sum of absolute differences

diff = F - F_8;
sum_diff = sum(abs(diff), 'all');

disp('Sum of Absolute Differences between Analytical F and F_8:');
disp(sum_diff);

%% Step 10
% Epipolar Geometry

% Draw 2D projections on image planes
cam1_fig = mvg_show_projected_points(cam1_p2d(1:2,:),imageSize,'Projected points on image plane 1 (F_8)');
cam2_fig = mvg_show_projected_points(cam2_p2d(1:2,:),imageSize,'Projected points on image plane 2 (F_8)');
% Draw epipolar lines
[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d,cam2_p2d,F_8);
[cam1_fig,cam2_fig] = mvg_show_epipolar_lines(cam1_fig, cam2_fig, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

% Compute epipoles from F_8
% Compute the epipoles by projecting the focal point of each camera to the image plane of the other

% Compute Camera Centers
% For Camera 1
[~, ~, V_P1] = svd(P1);
Center1 = V_P1(:, end);
Center1 = Center1 / Center1(4); % Normalize to homogeneous coordinates
% For Camera 2
[~, ~, V_P2] = svd(P2);
Center2 = V_P2(:, end);
Center2 = Center2 / Center2(4); % Normalize to homogeneous coordinates

% Project C2 into image 1
e1_proj = P1 * Center2;
e1_proj = e1_proj / e1_proj(3); % Normalize to homogeneous coordinates
% Project C1 into image 2
e2_proj = P2 * Center1;
e2_proj = e2_proj / e2_proj(3); % Normalize to homogeneous coordinates

% Display the epipoles
fprintf('Epipole in Image 1 (from projection):\n');
disp(e1_proj);
fprintf('Epipole in Image 2 (from projection):\n');
disp(e2_proj);
% Draw Epipoles
[~,~] = mvg_show_epipoles(cam1_fig, cam2_fig, e1_proj, e2_proj);

%% Step11
%Add Gaussian Noise

% Compute the standard deviation for the noise
noise_std = 1 / 1.96;

% Generate Gaussian noise for the points in the first image
noise_cam1 = noise_std * randn(size(cam1_p2d));
% Generate Gaussian noise for the points in the second image
noise_cam2 = noise_std * randn(size(cam2_p2d));

% Add the noise to the projected 2D points
cam1_p2d_noisy = cam1_p2d + noise_cam1;
cam2_p2d_noisy = cam2_p2d + noise_cam2;

%% Step12
% Repeat step 8 up to 10 with the noisy 2D points

% Prepare homogeneous coordinates for the noisy points
x1_noisy = [cam1_p2d_noisy; ones(1, size(cam1_p2d_noisy, 2))];
x2_noisy = [cam2_p2d_noisy; ones(1, size(cam2_p2d_noisy, 2))];

% Number of points
num_points = size(x1_noisy, 2);

% Construct matrix A using noisy points
A_noisy = [x1_noisy(1,:)'.*x2_noisy(1,:)', x1_noisy(2,:)'.*x2_noisy(1,:)', x2_noisy(1,:)', x1_noisy(1,:)'.*x2_noisy(2,:)', x1_noisy(2,:)'.*x2_noisy(2,:)', x2_noisy(2,:)', x1_noisy(1,:)', x1_noisy(2,:)', ones(num_points, 1)];

% Solve for F using SVD
[~, ~, V_noisy] = svd(A_noisy);
F8_noisy = reshape(V_noisy(:, end), 3, 3)';

% Enforce rank-2 constraint
[U_noisy, S_noisy, V_noisy] = svd(F8_noisy);
S_noisy(3, 3) = 0;
F8_noisy = U_noisy * S_noisy * V_noisy';

% Normalize the fundamental matrix
F8_noisy = F8_noisy / F8_noisy(3, 3);

% Display the estimated fundamental matrix
fprintf('Estimated Fundamental Matrix with Noisy Points (F_noisy):\n');
disp(F8_noisy);

% Compute differences
diff_F_analytical_noisy = F - F8_noisy;
sum_diff_analytical_noisy = sum(abs(diff_F_analytical_noisy), 'all');
diff_F_estimated_noisy = F_8 - F8_noisy;
sum_diff_estimated_noisy = sum(abs(diff_F_estimated_noisy), 'all');

% Display the differences
fprintf('Sum of absolute differences between analytical F and F_noisy:\n');
disp(sum_diff_analytical_noisy);
fprintf('Sum of absolute differences between estimated F_8 and F_noisy:\n');
disp(sum_diff_estimated_noisy);

% Visualize the noisy points
cam1_fig_noisyP = mvg_show_projected_points(cam1_p2d_noisy(1:2,:), imageSize, 'Noisy Projected Points on Image Plane 1(Noise1)');
cam2_fig_noisyP = mvg_show_projected_points(cam2_p2d_noisy(1:2,:), imageSize, 'Noisy Projected Points on Image Plane 2(Noise1)');
% Compute epipolar lines for the noisy points
[~,~,c1_l_coeff_noisyP,c2_l_coeff_noisyP] = mvg_compute_epipolar_geom_modif(cam1_p2d_noisy, cam2_p2d_noisy, F8_noisy);
% Draw epipolar lines on the images with noisy points
[cam1_fig_noisyP, cam2_fig_noisyP] = mvg_show_epipolar_lines(cam1_fig_noisyP, cam2_fig_noisyP, c1_l_coeff_noisyP, c2_l_coeff_noisyP, [-400,1;300,400],'y');

% Compute epipoles: Method1

% Compute epipoles for noisy data using projection matrices
P1_noisy = P1 + noise_std * randn(size(P1)); % Add noise to P1
P2_noisy = P2 + noise_std * randn(size(P2)); % Add noise to P2

% For Camera 1 (noisy)
[~, ~, V_P1_noisy] = svd(P1_noisy);       % Perform SVD on the noisy projection matrix for Camera 1
Center1_noisy = V_P1_noisy(:, end);    
Center1_noisy = Center1_noisy / Center1_noisy(4); 
% For Camera 2 (noisy)
[~, ~, V_P2_noisy] = svd(P2_noisy);       % Perform SVD on the noisy projection matrix for Camera 2
Center2_noisy = V_P2_noisy(:, end);      
Center2_noisy = Center2_noisy / Center2_noisy(4); % 

% Project the noisy center of Camera 2 into image 1 (epipole in Image 1)
e1_proj_noisy = P1_noisy * Center2_noisy; 
e1_proj_noisy = e1_proj_noisy / e1_proj_noisy(3); 
% Project the noisy center of Camera 1 into image 2 (epipole in Image 2)
e2_proj_noisy = P2_noisy * Center1_noisy; 
e2_proj_noisy = e2_proj_noisy / e2_proj_noisy(3); 

% Display the epipoles computed from noisy data
fprintf('Noisy Epipole in Image 1 (from projection):\n');
disp(e1_proj_noisy);
fprintf('Noisy Epipole in Image 2 (from projection):\n');
disp(e2_proj_noisy);
% Visualize the epipoles (if a visualization function is available)
[~,~] = mvg_show_epipoles(cam1_fig_noisyP, cam2_fig_noisyP, e1_proj_noisy, e2_proj_noisy);

% Compute epipoles: Method2
% To have a seperate plot of the epipolar geometry using the constraint

% Visualize the noisy points
cam1_fig_noisyC = mvg_show_projected_points(cam1_p2d_noisy(1:2,:), imageSize, 'Noisy Projected Points on Image Plane 1(Noise1)');
cam2_fig_noisyC = mvg_show_projected_points(cam2_p2d_noisy(1:2,:), imageSize, 'Noisy Projected Points on Image Plane 2(Noise1)');
% Compute epipolar lines for the noisy points
[~,~,c1_l_coeff_noisyC,c2_l_coeff_noisyC] = mvg_compute_epipolar_geom_modif(cam1_p2d_noisy, cam2_p2d_noisy, F8_noisy);
% Draw epipolar lines on the images with noisy points
[cam1_fig_noisyC, cam2_fig_noisyC] = mvg_show_epipolar_lines(cam1_fig_noisyC, cam2_fig_noisyC, c1_l_coeff_noisyC, c2_l_coeff_noisyC, [-400,1;300,400],'g');

% Compute epipoles using F_noisy
[~,~,V_noisy] = svd(F8_noisy);               % Perform Singular Value Decomposition (SVD) on F8_noisy
ep_1_noisy = V_noisy(:, end);                % The epipole in the first image corresponds to the last column of V
ep_1_noisy = ep_1_noisy / ep_1_noisy(end);   % Normalize the epipole
[~,~,V_noisy] = svd(F8_noisy');              % Perform SVD on the transpose of F8_noisy
ep_2_noisy = V_noisy(:, end);                % The epipole in the second image corresponds to the last column of V
ep_2_noisy = ep_2_noisy / ep_2_noisy(end);   % Normalize the epipole

% Display epipoles
fprintf('Noise1 = Epipole in Image 1 :\n');
disp(ep_1_noisy);
fprintf('Noise1 = Epipole in Image 2:\n');
disp(ep_2_noisy);
% Draw epipoles
[~,~] = mvg_show_epipoles(cam1_fig_noisyC, cam2_fig_noisyC, ep_1_noisy, ep_2_noisy);

%% STEP 13
% Add increased gaussian noise

% Compute the new standard deviation for the noise
noise_std = 2 / 1.96;

% Generate Gaussian noise for the points in the first image
noise_cam1 = noise_std * randn(size(cam1_p2d));
% Generate Gaussian noise for the points in the second image
noise_cam2 = noise_std * randn(size(cam2_p2d));
% Add the noise to the projected 2D points
cam1_p2d_noisy = cam1_p2d + noise_cam1;
cam2_p2d_noisy = cam2_p2d + noise_cam2;

% Visualize the noisy points
cam1_fig_noisy = mvg_show_projected_points(cam1_p2d_noisy(1:2,:), imageSize, 'Noisy Projected Points on Image Plane 1 (Noise2)');
cam2_fig_noisy = mvg_show_projected_points(cam2_p2d_noisy(1:2,:), imageSize, 'Noisy Projected Points on Image Plane 2 (Noise2)');

%% Repeat Steps 8 to 12

% Prepare homogeneous coordinates for the noisy points
x1_noisy = [cam1_p2d_noisy; ones(1, size(cam1_p2d_noisy, 2))];
x2_noisy = [cam2_p2d_noisy; ones(1, size(cam2_p2d_noisy, 2))];

% Number of points
num_points = size(x1_noisy, 2);

% Construct matrix A using noisy points
A_noisy = [x1_noisy(1,:)'.*x2_noisy(1,:)', x1_noisy(2,:)'.*x2_noisy(1,:)', x2_noisy(1,:)', x1_noisy(1,:)'.*x2_noisy(2,:)', x1_noisy(2,:)'.*x2_noisy(2,:)', x2_noisy(2,:)', x1_noisy(1,:)', x1_noisy(2,:)', ones(num_points, 1)];

% Solve for F using SVD
[~, ~, V_noisy] = svd(A_noisy);
F8_noisy = reshape(V_noisy(:, end), 3, 3)';

% Enforce rank-2 constraint
[U_noisy, S_noisy, V_noisy] = svd(F8_noisy);
S_noisy(3, 3) = 0;
F8_noisy = U_noisy * S_noisy * V_noisy';

% Normalize the fundamental matrix
F8_noisy = F8_noisy / F8_noisy(3, 3);

% Display the estimated fundamental matrix
fprintf('Estimated Fundamental Matrix with Noisy Points (F_noisy):\n');
disp(F8_noisy);

% Compute differences
diff_F_analytical_noisy = F - F8_noisy;
sum_diff_analytical_noisy = sum(abs(diff_F_analytical_noisy), 'all');
diff_F_estimated_noisy = F_8 - F8_noisy;
sum_diff_estimated_noisy = sum(abs(diff_F_estimated_noisy), 'all');

% Display the differences
fprintf('Sum of absolute differences between analytical F and F_noisy:\n');
disp(sum_diff_analytical_noisy);
fprintf('Sum of absolute differences between estimated F_8 and F_noisy:\n');
disp(sum_diff_estimated_noisy);

% Compute epipolar lines for the noisy points
[~,~,c1_l_coeff_noisy,c2_l_coeff_noisy] = mvg_compute_epipolar_geom_modif(cam1_p2d_noisy, cam2_p2d_noisy, F8_noisy);

% Draw epipolar lines on the images with noisy points
[cam1_fig_noisy, cam2_fig_noisy] = mvg_show_epipolar_lines(cam1_fig_noisy, cam2_fig_noisy, c1_l_coeff_noisy, c2_l_coeff_noisy, [-400,1;300,400],'b');

% Compute epipoles using F_noisy
[~,~,V_noisy] = svd(F8_noisy);
ep_1_noisy = V_noisy(:, end);
ep_1_noisy = ep_1_noisy / ep_1_noisy(end);

[~,~,V_noisy] = svd(F8_noisy');
ep_2_noisy = V_noisy(:, end);
ep_2_noisy = ep_2_noisy / ep_2_noisy(end);

% Display epipoles
fprintf(' Noise2 = Epipole in Image 1:\n');
disp(ep_1_noisy);
fprintf('Noise2 = Epipole in Image 2:\n');
disp(ep_2_noisy);

% Plot epipoles 
[~,~] = mvg_show_epipoles(cam1_fig_noisy, cam2_fig_noisy, ep_1_noisy, ep_2_noisy);

%% Step 14: 
% Normalization Step for increased noisy points

% Prepare homogeneous coordinates for the noisy points
x1_noisy = [cam1_p2d_noisy(1:2, :); ones(1, size(cam1_p2d_noisy, 2))];
x2_noisy = [cam2_p2d_noisy(1:2, :); ones(1, size(cam2_p2d_noisy, 2))];

% Normalize points in image 1
[T1,x1_normalized] = normalize_points(x1_noisy);
% Normalize points in image 2
[T2,x2_normalized] = normalize_points(x2_noisy);

% Construct Matrix A_hat using Normalized Points
num_points = size(x1_normalized, 2);
A_hat_noisy = [x1_normalized(1,:)' .* x2_normalized(1,:)', ...
              x1_normalized(2,:)' .* x2_normalized(1,:)', ...
              x2_normalized(1,:)', ...
              x1_normalized(1,:)' .* x2_normalized(2,:)', ...
              x1_normalized(2,:)' .* x2_normalized(2,:)', ...
              x2_normalized(2,:)', ...
              x1_normalized(1,:)', ...
              x1_normalized(2,:)', ...
              ones(num_points, 1)];

% Compute SVD of A_hat
[~, ~, V] = svd(A_hat_noisy);
% Extract the fundamental matrix from the last column of V_Ahat
F_hat_noisy = reshape(V(:, end), 3, 3)';

% Enforce Rank-2 Constraint on F_hat
[U, S, V] = svd(F_hat_noisy);
S(3, 3) = 0;  % Set the smallest singular value to zero
F_hat_noisy_rank2 = U * S * V';

% Denormalize the fundamental matrix
F_normalized_noisy = T2' * F_hat_noisy_rank2 * T1;
F_normalized_noisy = F_normalized_noisy / F_normalized_noisy(3, 3);
% Display the estimated fundamental matrix
fprintf('Estimated Noisy Fundamental Matrix using Normalized Eight-Point Algorithm (F_normalized_noisy):\n');
disp(F_normalized_noisy);

% Comparison of Fundamental Matrices

% Difference between the analytical F and the normalized noisy F
diff_F_analytical_normalized = F - F_normalized_noisy;
sum_diff_analytical_normalized = sum(abs(diff_F_analytical_normalized), 'all');
% Difference between F_8 (from Step 8 without normalization) and the normalized noisy F
diff_F_estimated_normalized = F_8 - F_normalized_noisy;
sum_diff_estimated_normalized = sum(abs(diff_F_estimated_normalized), 'all');

% Display the differences
fprintf('Sum of absolute differences between analytical F and F_normalized_noisy:\n');
disp(sum_diff_analytical_normalized);

fprintf('Sum of absolute differences between F_8 and F_normalized_noisy:\n');
disp(sum_diff_estimated_normalized);

% Recompute Epipolar Lines and Epipoles with Normalized Noisy F

% Visualize the noisy points
cam1_fig_norm = mvg_show_projected_points(x1_noisy(1:2,:), imageSize, 'Noisy Projected Points on Image Plane 1 (Normalized)');
cam2_fig_norm = mvg_show_projected_points(x2_noisy(1:2,:), imageSize, 'Noisy Projected Points on Image Plane 2 (Normalized)');

% Compute epipolary lines for the noisy points
[~,~,c1_l_coeff_norm,c2_l_coeff_norm] = mvg_compute_epipolar_geom_modif(x1_noisy, x2_noisy, F_normalized_noisy);

% Draw epipolar lines on the images with noisy points
[cam1_fig_norm, cam2_fig_norm] = mvg_show_epipolar_lines(cam1_fig_norm, cam2_fig_norm, c1_l_coeff_norm, c2_l_coeff_norm, [-400,1;300,400], 'm'); 

% Compute epipoles using F_normalized_noisy
[~, ~, V_ep1_noisy] = svd(F_normalized_noisy);
ep_1_noisy = V_ep1_noisy(:, end);
ep_1_noisy = ep_1_noisy / ep_1_noisy(3);
[~, ~, V_ep2_noisy] = svd(F_normalized_noisy');
ep_2_noisy = V_ep2_noisy(:, end);
ep_2_noisy = ep_2_noisy / ep_2_noisy(3);

% Display the epipoles on the images with noisy points
[~,~] = mvg_show_epipoles(cam1_fig_norm, cam2_fig_norm, ep_1_noisy, ep_2_noisy); 

% Compute the condition number for the matrix without normalization 
cond_A_noisy = cond(A_noisy); 
fprintf('Condition Number without Normalization: %f\n',cond_A_noisy);

% Compute the condition number for the matrix with normalization 
cond_A_norm_noisy = cond(A_hat_noisy); 
fprintf('Condition Number with Normalization: %f\n', cond_A_norm_noisy);




