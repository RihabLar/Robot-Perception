function [T, normalized_points] = normalize_points(points)
    % Compute the centroid of the points
    centroid = mean(points(1:2,:), 2);
    
    % Translate points to have the centroid at the origin
    translated_points = points(1:2,:) - centroid;
    
    % Compute the average distance from the origin
    avg_dist = mean(sqrt(sum(translated_points.^2, 1)));
    
    % Scaling factor to make the average distance sqrt(2)
    scale = sqrt(2) / avg_dist;
    
    % Construct the normalization matrix
    T = [scale, 0, -scale*centroid(1);
         0, scale, -scale*centroid(2);
         0, 0, 1];
    
    % Normalize the points
    normalized_points = T * points;
end

