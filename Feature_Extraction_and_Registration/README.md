# Feature Extraction and Image Registration

This project explores **SIFT (Scale-Invariant Feature Transform)** and **homography estimation** for rigid image registration.  
The goal is to extract invariant features, match them across images, and compute transformations to align image pairs.

## Overview
- Implemented **SIFT feature extraction** to detect keypoints and descriptors.  
- Performed **feature association** using Lowe’s distance ratio test.  
- Tested different thresholds (`distRatio = 0.4, 0.6, 0.8`) to analyze the trade-off between number of matches and accuracy.  
- Estimated **homographies** (Translation, Similarity, Affine, Projective) to align image pairs.  
- Evaluated accuracy using **reprojection error**.  
- Improved robustness with **RANSAC** to reject outliers before computing homographies.

## Key Features
- Feature extraction and descriptor matching with SIFT.  
- Homography estimation under different motion models.  
- Visualization of matched features and reprojection errors.  
- RANSAC-based outlier rejection for improved registration accuracy.  

## Files
- `matchsiftmodif.m` – Modified SIFT matching with distance ratio test  
- `computeHomography.m` – Homography estimation for different models  
- `computeHomographyRANSAC.m` – RANSAC-based homography estimation  
- `projectionerrorvec.m` – Compute reprojection error vector  
- `visualizeReprojectionError.m` – Visualize reprojection errors  
- `Lab2_DavinaSanghera_RihabLaroussi.pdf` – Full project report  

## Results
- Lower `distRatio` values yield fewer but more accurate matches.  
- Higher `distRatio` increases matches but also false positives.  
- Projective transformations provided the best alignment for complex cases, while similarity/affine were more efficient for simpler ones.  
- RANSAC significantly reduced reprojection errors by filtering outliers.  

## Conclusion
This project demonstrates the effectiveness of **local feature-based methods** for image registration.  
By combining SIFT, homography estimation, and RANSAC, we achieved robust alignment even in noisy, real-world images.
