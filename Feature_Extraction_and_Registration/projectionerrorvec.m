
% This function calculates the Euclidean distance (error) between each 
% point in CL1uv and its corresponding projected point in CL2uv usinG a given homography matrix H12.
    
function errorVec = projectionerrorvec(H12, CL1uv, CL2uv)

n = size(CL1uv, 1);          % Number of points in CL1uv (and CL2uv)

errorVec = zeros(n,1);       % Initialize error vector to store distances

for i = 1:n 

    projection_p = H12 * [CL2uv(i,:), 1]';                   % Convert to homogeneous coordinates and apply H12
    projection_p = projection_p(1:2) / projection_p(3);      % Normalize the projected point to non-homogeneous coordinates
    errorVec(i) = norm(CL1uv(i,:) - projection_p');          % Compute Euclidean distance (error) between the original point in CL1uv 
                                                             % and the projected point in CL2uv

end

end 

