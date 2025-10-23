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

