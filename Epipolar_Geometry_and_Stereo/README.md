# Epipolar Geometry and Stereo

This project explores **epipolar geometry** in stereo vision, focusing on the computation of the **fundamental matrix (F)** and its role in relating corresponding points between two camera views.  
We implemented both **analytical** and **algorithmic** approaches, studied the effects of noise, and applied normalization to improve robustness.

## Overview
- **Analytical Computation**: Derived the fundamental matrix using known intrinsic and extrinsic parameters of two cameras.  
- **8-Point Algorithm**: Implemented the classic algorithm to estimate F from point correspondences, enforcing the rank-2 constraint.  
- **Epipolar Geometry**: Computed and visualized epipolar lines and epipoles for stereo image pairs.  
- **Noise Analysis**: Added Gaussian noise to image points to evaluate robustness of F estimation.  
- **Normalization**: Applied point normalization to improve numerical stability and accuracy.  

## Results
- Analytical and 8-point F matrices matched closely in noise-free conditions.  
- With Gaussian noise, deviations appeared but epipolar geometry remained valid.  
- Higher noise levels degraded accuracy, shifting epipoles and distorting lines.  
- Normalization improved numerical stability and reduced error.  


