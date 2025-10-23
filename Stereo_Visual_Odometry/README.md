# Stereo Visual Odometry

This project implements **stereo visual odometry (VO)** using a dataset of stereo image pairs from the UTIAS Long-Term Localization and Mapping dataset.  
The goal is to estimate the motion of a stereo camera system over time using feature extraction, stereo matching, and motion estimation methods.

## Overview
- **Feature Extraction & Bucketing**: Extracted features uniformly across the image using bucketing to avoid clustering in feature-rich regions.  
- **Circular Matching**: Matched features across stereo pairs and consecutive frames by enforcing loop closure.  
- **Motion Estimation**: Implemented and compared two methods:
  - **3D–3D Method**: Aligns two sets of triangulated 3D points from consecutive stereo pairs.  
  - **2D–3D Method**: Solves the Perspective-n-Point (PnP) problem using 2D features and 3D points.  
- **Monte Carlo Analysis**: Added Gaussian noise to evaluate robustness of both methods over 1000 runs.  
- **Trajectory Estimation**: Processed a sequence of 981 stereo pairs to estimate the camera trajectory and aligned it with GPS ground truth.  

## Key Features
- Depth filtering with `select_based_on_z_distance`.  
- Monte Carlo sensitivity analysis with `mvg_montecarlo_3d_to_3d_reg` and `mvg_montecarlo_2d_to_3d_reg`.  
- Visualization of VO trajectory vs. GPS trajectory.  
- Discussion of improvements such as RANSAC-based outlier rejection, bundle adjustment, and adaptive depth limits.  

## Files
- `Stereo Visual Odometry.pdf` – Full project report with methodology and results.  
- `select_based_on_z_distance.m` – Depth filtering function.  
- `mvg_montecarlo_3d_to_3d_reg.m` – Monte Carlo analysis for 3D–3D method.  
- `mvg_montecarlo_2d_to_3d_reg.m` – Monte Carlo analysis for 2D–3D method.  
- `main_vo_script.m` – Main script for stereo VO pipeline.  

## Results
- **3D–3D method** was less robust due to compounded triangulation errors.  
- **2D–3D method** produced more consistent translations and was more resilient to noise.  
- Monte Carlo analysis confirmed the 2D–3D method’s superiority, with tighter translation clusters.  
- VO trajectory aligned reasonably with GPS, though drift was observed.  

## Conclusion
Stereo visual odometry provides a practical way to estimate robot motion from stereo images.  
The **2D–3D method** proved more robust than the 3D–3D method.  
Future improvements include integrating RANSAC earlier in the pipeline, applying bundle adjustment, and dynamically adjusting depth limits for more accurate and drift-free trajectories.
