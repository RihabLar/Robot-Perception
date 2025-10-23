
% Step 1: Load Images
I1 = imread('00.png'); % Reference image (base image)
I2 = imread('01.png'); % Target image (the one to be transformed)

% Step 2: Load Feature Coordinates
load('Features.mat'); % Ensure this loads CL1uv and CL2uv for the image pair

% Step 3: Compute Homography
% Specify the model type here (e.g., 'Translation', 'Similarity', 'Affine', or 'Projective')
Model = 'Projective';  % Change this to test different models
H12 = computeHomography(CL1uv, CL2uv, Model);

% Step 4: Display the Warped Overlay
overlay = showwarpedimages(I1, I2, H12);