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

## Results
- Lower `distRatio` values yield fewer but more accurate matches.  
- Higher `distRatio` increases matches but also false positives.  
- Projective transformations provided the best alignment for complex cases, while similarity/affine were more efficient for simpler ones.  
- RANSAC significantly reduced reprojection errors by filtering outliers.  
