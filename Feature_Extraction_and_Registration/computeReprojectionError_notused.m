function avgError = computeReprojectionError(H12, CL1uv, CL2uv)
    n = size(CL1uv, 1);
    totalError = 0;

    for i = 1:n
        % Original point in CL1uv
        point1 = [CL2uv(i, :), 1]';
        
        % Reprojected point in CL2uv
        projectedPoint = H12 * point1;
        projectedPoint = projectedPoint / projectedPoint(3);  % Normalize to homogeneous coordinates
        
        % Corresponding point in CL2uv
        point2 = CL1uv(i, :)';
        
        % Compute Euclidean distance (reprojection error)
        error = norm(projectedPoint(1:2) - point2);
        totalError = totalError + error;
    end

    % Average reprojection error
    avgError = totalError / n;
end
