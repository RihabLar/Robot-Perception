function visualizeReprojectionError(I1, I2, H12, CL1uv, CL2uv)
    figure;
    subplot(1, 2, 1);
    imshow(I1);
    hold on;
    plot(CL1uv(:,1), CL1uv(:,2), 'go'); % Original points in green
    title('Original Image with Points');

    subplot(1, 2, 2);
    imshow(I2);
    hold on;
    
    n = size(CL1uv, 1);
    errors = zeros(n, 1);
    
    for i = 1:n
        % Original point in CL2uv
        point1 = [CL2uv(i, :), 1]';
        % Reprojected point in CL2uv
        projectedPoint = H12 .* point1;
        projectedPoint = projectedPoint / projectedPoint(3);
        % Corresponding point in CL1uv
        point2 = CL1uv(i, :)';
        % Compute Euclidean distance (reprojection error)
        error = norm(projectedPoint(1:2) - point2);
        errors(i) = error;

        plot(projectedPoint(1), projectedPoint(2), 'go'); % Projected points in green
        plot(point2(1), point2(2), 'ro'); % Corresponding points in red

        line([projectedPoint(1), point2(1)], [projectedPoint(2), point2(2)], 'Color', 'y'); % Error line in yellow
    end
    
    avgError = mean(errors);
    title(sprintf('Reprojected Image with Errors\nAverage Reprojection Error: %.4f', avgError));
    
    % Print the numerical result to the console
    fprintf('Average Reprojection Error: %.4f\n', avgError);
end
