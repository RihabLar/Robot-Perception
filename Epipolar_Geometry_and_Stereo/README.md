# Epipolar Geometry and Stereo

This project explores **epipolar geometry** in stereo vision, focusing on the computation of the **fundamental matrix (F)** and its role in relating corresponding points between two camera views.  
We implemented both **analytical** and **algorithmic** approaches, studied the effects of noise, and applied normalization to improve robustness.

## Overview
- **Analytical Computation**: Derived the fundamental matrix using known intrinsic and extrinsic parameters of two cameras.  
- **8-Point Algorithm**: Implemented the classic algorithm to estimate F from point correspondences, enforcing the rank-2 constraint.  
- **Epipolar Geometry**: Computed and visualized epipolar lines and epipoles for stereo image pairs.  
- **Noise Analysis**: Added Gaussian noise to image points to evaluate robustness of F estimation.  
- **Normalization**: Applied point normalization to improve numerical stability and accuracy.  

## Key Features
- Projection matrices for stereo cameras.  
- Fundamental matrix estimation (analytical vs. 8-point algorithm).  
- Epipolar line and epipole visualization.  
- Noise modeling and its effect on F and epipolar geometry.  
- Normalization for stable computation.  

## Files
- `Lab3_RihabLaroussi_DavinaSanghera.pdf` – Full project report with methodology and results.  
- `eight_point_algorithm.m` – Implementation of the 8-point algorithm.  
- `compute_fundamental_matrix.m` – Analytical computation of F.  
- `normalize_points.m` – Normalization of image coordinates.  
- `mvg_compute_epipolar_geom_modif.m` – Compute epipolar line coefficients.  
- `mvg_show_epipolar_lines.m` – Visualize epipolar lines.  
- `mvg_show_epipoles.m` – Display epipoles on image planes.  

## Results
- Analytical and 8-point F matrices matched closely in noise-free conditions.  
- With Gaussian noise, deviations appeared but epipolar geometry remained valid.  
- Higher noise levels degraded accuracy, shifting epipoles and distorting lines.  
- Normalization improved numerical stability and reduced error.  

## Conclusion
This project demonstrates how **epipolar geometry** underpins stereo vision.  
By combining analytical derivations, the 8-point algorithm, and normalization, we achieved robust estimation of the fundamental matrix and accurate visualization of epipolar constraints, even under noisy conditions.
