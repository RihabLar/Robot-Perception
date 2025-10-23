function [normalised_pts] = normalise(pts)
% Normalize points first by translating the points so they are centred
% around the origin then by scaling the points so that they have the same
% average distance to the origin

    % Translate points by the centre
    centre = mean(pts, 1);
    centered_pts = pts - centre;

    % Compute the mean distance to the centre
    distances = sqrt(sum(centered_pts.^2, 2));
    mean_dist = mean(distances);
    
    % Scale factor to get an mean distance of sqrt(2)
    scale = sqrt(2) / mean_dist;
    
    % Normalise the points
    normalised_pts = centered_pts * scale;
end