# Camera Calibration

This project implements **camera calibration** for a simulated camera, exploring how 3D world points project onto a 2D image plane using intrinsic and extrinsic parameters.  
The goal is to understand projection geometry and evaluate calibration accuracy under different conditions.

## Overview
- Defined **intrinsic parameters** (focal lengths, principal point, skew).  
- Defined **extrinsic parameters** (rotation and translation between world and camera frames).  
- Constructed the **camera projection matrix**.  
- Projected 3D points into the image plane and visualized results.  
- Implemented the **Hall method** to estimate the projection matrix from 2D–3D correspondences.  
- Added **Gaussian noise** to assess calibration robustness.  
- Analyzed the effect of increasing the number of 3D points on projection error.

## Results
- Without noise, the Hall method recovered the exact projection matrix.  
- With Gaussian noise, intrinsic parameters shifted slightly, and skew became non-zero.  
- Increasing the number of 3D points reduced projection error, confirming that more data improves calibration accuracy.


