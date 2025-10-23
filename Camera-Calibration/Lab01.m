%--------------------------------STEP 1-----------------------------------%

% Intrinsic parameters
au = 557.0943; av = 712.9824;
u0 = 326.3819; v0 = 298.6679;

% Location of the world frame in camera coordinates (in mm)
Tx = 100; Ty = 0; Tz = 1500;

% World rotation w.r.t camera coordinates
% Euler XYX1 angles
Phix = 0.8*pi/2;
Phiy = -1.8*pi/2;
Phix1 = pi/5;

%--------------------------------STEP 2-----------------------------------%

% Intrinsic matrix K
K = [au 0 u0; 0 av v0; 0 0 1];

% Rotation matrix cRw
rot_x = [1 0 0; 0 cos(Phix) -sin(Phix); 0 sin(Phix) cos(Phix)];
rot_y = [cos(Phiy) 0 sin(Phiy); 0 1 0; -sin(Phiy) 0 cos(Phiy)];
rot_x1 = [1 0 0; 0 cos(Phix1) -sin(Phix1); 0 sin(Phix1) cos(Phix1)];
cRw = rot_x * rot_y * rot_x1;

% Translation matrix ctw
ctw = [100; 0; 1500];

% Transformation matrix cTw
cTw = [cRw ctw; 0 0 0 1];

% Projection matrix
P = K * [1 0 0 0; 0 1 0 0; 0 0 1 0] * cTw;

%--------------------------------STEP 3-----------------------------------%

% Generate set of 6 3D points
pts_3d = generate_hom_3d_pts(6, -480, 480);

%--------------------------------STEP 4-----------------------------------%

% Project 3D points onto image plane
pts_3d_proj = project_3d_to_image_plane(pts_3d, P);

% Convert to 2D
pts_2d = convert_to_2d(pts_3d_proj);

%--------------------------------STEP 5-----------------------------------%

% Plot 2d points
x = pts_2d(:, 1);
y = pts_2d(:, 2);
plot(x, y, 'o');
title('2D Points Projection'); 


%--------------------------------STEP 6-----------------------------------%

% Hall method to compute P
A = hall_method(pts_2d, pts_3d)

% Normalise P by its scale factor
P_normalised = P/P(3,4)

%--------------------------------STEP 7-----------------------------------%

% Extract intrinsic parameters
[K, cRw] = get_intrinsics_from_proj_matrix(P)

%--------------------------------STEP 8-----------------------------------%

% Add Gaussian noise to 2D points
pts_2d_noisy = add_Gaussian_noise(pts_2d);

% Hall method to compute noisy P
A_noisy = hall_method(pts_2d_noisy, pts_3d);

% Extract intrinsic parameters
[K_noisy, cRw_noisy] = get_intrinsics_from_proj_matrix(A_noisy)

%--------------------------------STEP 9-----------------------------------%

% Project 3D points onto image plane using noisy P
pts_3d_proj_noisy = project_3d_to_image_plane(pts_3d, A_noisy);

% Convert to 2D
pts_2d_noisy = convert_to_2d(pts_3d_proj_noisy);

% Compute average projection error
avg_proj_error = average_projection_error(pts_2d, pts_2d_noisy)

%--------------------------------STEP 10----------------------------------%

% Computing the average projection error for 10 points
pts_3d_10 = generate_hom_3d_pts(10, -480, 480);
pts_3d_proj_10 = project_3d_to_image_plane(pts_3d_10, P);
pts_2d_10 = convert_to_2d(pts_3d_proj_10);
pts_2d_noisy_10 = add_Gaussian_noise(pts_2d_10);
A_noisy_10 = hall_method(pts_2d_noisy_10, pts_3d_10);
pts_3d_proj_noisy_10 = project_3d_to_image_plane(pts_3d_10, A_noisy_10);
pts_2d_noisy_10 = convert_to_2d(pts_3d_proj_noisy_10);
avg_proj_error_10 = average_projection_error(pts_2d_10, pts_2d_noisy_10)

% Computing the average projection error for 50 points
pts_3d_50 = generate_hom_3d_pts(50, -480, 480);
pts_3d_proj_50 = project_3d_to_image_plane(pts_3d_50, P);
pts_2d_50 = convert_to_2d(pts_3d_proj_50);
pts_2d_noisy_50 = add_Gaussian_noise(pts_2d_50);
A_noisy_50 = hall_method(pts_2d_noisy_50, pts_3d_50);
pts_3d_proj_noisy_50 = project_3d_to_image_plane(pts_3d_50, A_noisy_50);
pts_2d_noisy_50 = convert_to_2d(pts_3d_proj_noisy_50);
avg_proj_error_50 = average_projection_error(pts_2d_50, pts_2d_noisy_50)

% Plot average projection error against number of points
avg_proj_errors = [avg_proj_error, avg_proj_error_10, avg_proj_error_50];
no_of_points = [6, 10, 50];
figure;
plot(no_of_points, avg_proj_errors, '-o');
title('Average Projection Error vs Number of Points');
xlabel('Number of Points');
ylabel('Average Projection Error');

%-------------------------------------------------------------------------%


function [pts_hom_3d] = generate_hom_3d_pts(n, lower, upper)
% Randomly generate a set of n homogeneous 3d points within the range (lower, upper)
pts_cart_3d = randi([lower, upper], n, 3);
pts_hom_3d = [pts_cart_3d, ones(n, 1)];
end

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

function [pts_3d_proj] = project_3d_to_image_plane(pts_3d, P)
% Multiply each 3D point in pts_3d by P
n = length(pts_3d);
pts_3d_proj= [];
for i = 1:n
    pt_proj = P * pts_3d(i, :)';
    pts_3d_proj = [pts_3d_proj; pt_proj'];
end
end

function [A] = hall_method(pts_2d, pts_3d)
% Hall method to compute P
Q = [];
B = [];
n = length(pts_2d);

for i = 1:n
    Q1 = [pts_3d(i, 1), pts_3d(i, 2), pts_3d(i, 3), 1, 0, 0, 0, 0, -pts_2d(i, 1)*pts_3d(i, 1), -pts_2d(i, 1)*pts_3d(i, 2), -pts_2d(i, 1)*pts_3d(i, 3)];
    Q2 = [0, 0, 0, 0, pts_3d(i, 1), pts_3d(i, 2), pts_3d(i, 3), 1, -pts_2d(i, 2)*pts_3d(i, 1), -pts_2d(i, 2)*pts_3d(i, 2), -pts_2d(i, 2)*pts_3d(i, 3)];
    Q = [Q; Q1; Q2];
    B1 = pts_2d(i, 1);
    B2 = pts_2d(i, 2);
    B = [B; B1; B2];
end

% Obtain projection matrix
A_vector = Q\B;
A = [A_vector(1), A_vector(2), A_vector(3), A_vector(4); A_vector(5), A_vector(6), A_vector(7), A_vector(8); A_vector(9), A_vector(10), A_vector(11), 1];
end

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

function [K,cRw] = get_intrinsics_from_proj_matrix(P)
% Get left-side 3x3 block of P
M = P(:,1:3);
% This implements the RQ decomposition from QR in matlab
[Q,R] = qr(rot90(M,3));
R = rot90(R,2)';
Q = rot90(Q);
% Check the determinant of Q to make cRw a proper rotation
if det(Q) < 0
cRw = -Q;
else
cRw = Q;
end
% Get the normalized intrinsics
K = R/R(3,3);
end

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
