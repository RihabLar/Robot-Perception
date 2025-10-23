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

## Results
- **3D–3D method** was less robust due to compounded triangulation errors.  
- **2D–3D method** produced more consistent translations and was more resilient to noise.  
- Monte Carlo analysis confirmed the 2D–3D method’s superiority, with tighter translation clusters.  
- VO trajectory aligned reasonably with GPS, though drift was observed.  
