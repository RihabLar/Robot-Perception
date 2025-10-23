function visualizeReprojectionErrorWithRANSAC(I1, I2, CL1uv, CL2uv, Model)
    % Perform RANSAC to find the inliers and the homography matrix
    [H12, INLIERS1uv, INLIERS2uv] = computeHomographyRANSAC(CL1uv, CL2uv, Model);

    figure;
    subplot(1, 2, 1);
    imshow(I1);
    hold on;
    plot(CL1uv(:,1), CL1uv(:,2), 'go'); % Original points in green
    plot(INLIERS1uv(:,1), INLIERS1uv(:,2), 'bo'); % Inliers in blue
    title('Original Image with Points');

    subplot(1, 2, 2);
    imshow(I2);
    hold on;
    
    n = size(INLIERS1uv, 1);
    errors = zeros(n, 1);
    
    for i = 1:n
        point1 = [INLIERS1uv(i, :), 1]';
        projectedPoint = H12 * point1;
        projectedPoint = projectedPoint / projectedPoint(3);
        
        point2 = INLIERS2uv(i, :)';
        error = norm(projectedPoint(1:2) - point2);
        errors(i) = error;

        plot(projectedPoint(1), projectedPoint(2), 'go'); % Projected points in green
        plot(point2(1), point2(2), 'ro'); % Corresponding points in red

        line([projectedPoint(1), point2(1)], [projectedPoint(2), point2(2)], 'Color', 'y'); % Error line in yellow
    end
    
    avgError = mean(errors);
    numInliers = length(INLIERS1uv);
    title(sprintf('Reprojected Image with Errors\nAvg Reprojection Error: %.4f\nNumber of Inliers: %d', avgError, numInliers));
    
    % Print the numerical result to the console
    fprintf('Average Reprojection Error (Inliers): %.4f\n', avgError);
    fprintf('Number of Inliers: %d\n', numInliers);
end
