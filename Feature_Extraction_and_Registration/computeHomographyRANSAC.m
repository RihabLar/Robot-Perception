function [H12, INLIERS1uv, INLIERS2uv] = computeHomographyRANSAC(CL1uv, CL2uv, Model)
% computeHomographyRANSAC : perform RANSAC to remove outliers then estimate the Homography between two images 
    
    % Initialise variables
    dist_threshold = 150; % distance threshold
    sample_size = 10; % sample size
    no_of_trials = 100; % number of trials
    term_threshold = (1 - 0.2) * no_of_trials; % termination threshold
    no_of_points = height(CL1uv);  % number of points
    
    % Ensure sample size is not greater than the number of points
    if sample_size > no_of_points
        sample_size = no_of_points;
    end
    
    best_consensus_idxs = [];
    best_consensus_size = 0;
    best_H12 = eye(3);
    
    for i = 1:no_of_trials
        % Randomly sample points
        sample_idxs = randperm(no_of_points, sample_size);
        sample1 = CL1uv(sample_idxs, :);
        sample2 = CL2uv(sample_idxs, :);
        
        % Estimate H12
        H12 = computeHomography(sample1, sample2, Model);
        
        % Calculate projection error of each correspondence
        errorVec = projectionerrorvec(H12, CL1uv, CL2uv);
        
        % Inliers are those with an error below the distance threshold
        inliers_idxs = find(errorVec < dist_threshold);
        
        % Check if this is the largest consensus 
        % and if the current model is above the termination threshold
        consensus_size = length(inliers_idxs);
        if consensus_size > best_consensus_size
            best_consensus_idxs = inliers_idxs;
            best_consensus_size = consensus_size;
            best_H12 = H12;
        end
        if (consensus_size / no_of_points) >= term_threshold
            break
        end
    end
    
    % Separate inliers and outliers based on the best consensus set found
    H12 = best_H12;
    INLIERS1uv = CL2uv(best_consensus_idxs, :);
    INLIERS2uv = CL1uv(best_consensus_idxs, :);
end
