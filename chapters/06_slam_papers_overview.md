# Overview of Various SLAM-related Papers

## 1. Classic Papers

### 1.1 Systems based on Direct Method

#### 1.1.1 SVO - Fast Semi-Direct Monocular Visual Odometry [@Forster2014]

- [Paper](https://infoscience.epfl.ch/record/199740/files/ICRA14_Forster.pdf)


## 2. New Papers (2019)

### 2.1 ICRA 2019

#### 2.1.1 Loosely-Coupled Semi-Direct Monocular SLAM [@SHLee2019]

- [Paper](https://ieeexplore.ieee.org/abstract/document/8584894)

Features:

- Combine the complementary strength of direct and feature-based methods
- 2 modules running in parallel: 
  1) Direct odometry module: uses direct method to track new frames based on a 
     local semi-dense map
  2) Feature-based module: builds a sparse feature-based map based on tracked 
     map points and poses
- 3 levels of parallel optimizations
  1) Photometric bundle adjustment (BA): 
     jointly optimizes local structure and motion
  2) Geometric BA: refines keyframe poses and associated feature map points
  3) Pose graph optimization: achieve global map consistency in case of
     loop closures
- Achieve real-time by limiting feature-based operations to marginalized 
  keyframes from the direct odometry module

System diagram:  
![System Diagram](images/ch06/shlee2019_fig_02.jpg){ width=100% }
- Built upon direct sparse odometry (DSO) & ORB-SLAM

#### 2.1.2 Sparse2Dense: From Direct Sparse Odometry to Dense 3-D Reconstruction [@Tang2019]

- [Paper](https://ieeexplore.ieee.org/document/8605349)

Features:

- A *deep* dense monocular SLAM system
- Construct a dense 3D model via a sparse to dense mapping using 
  learned surface normals  
  ![Sparse2Dense](images/ch06/tang2019_fig_03.jpg){ width=100% }
- Depth estimations are learned in single view as prior for monocular 
  visual odometry (VO)
- VO with the prior above can obtain both accurate positioning and high-quality
  depth reconstruction
- Depth and normals are predicted by a single convoluted neural network (CNN) 
  trained in a tightly coupled manner
  - Depth: used in the projective geometric optimization for accurate 
    pose estimation
  - Normal: used for a dense geometrical reconstruction with intermediate
    sparse point clouds
  - CNN structure  
    ![FCRN vs. FCDRN](images/ch06/tang2019_fig_04.jpg){ width=50% }
    - Left: original *fully convolutional residual network* (FCRN) 
      with an encoder-decoder structure based on *ResNet-50*
    - right: FCDRN with *Resnet-50* replaced by the *Dilated Residual Network* 
      (DRN)

System diagram:  
![System Diagram](images/ch06/tang2019_fig_02.jpg){ width=100% }

- 4 major stages:
  1) Depth/normal generation using CNN
  2) Visual tracking using direct alignment (direct sparse odometry, DSO)
  3) Geometrical sparse to dense reconstruction
  4) Fusion-based mapping
- Contribution on stage 1 & 3

#### 2.1.3 Tightly-Coupled Visual-Inertial Localization and 3-D Rigid-Body Target Tracking [@Eckenhoff2019]

- [Paper](https://ieeexplore.ieee.org/document/8629939)

Features:

- Tightly-coupled estimator for visual-inertial localization and target tracking
  (VILTT)
  - Buiit upon visual-inertial navigation system (VINS) which is based on 
    multi-state constraint Kalman filter (MSCKF) framework
  - Generalize the VINS to incorporate a 6DOF rigid-body target tracking
    of a 3D moving object
- Key contribution: tightly-coupling of target motion estimation within VINS
- Represent the target object as a 3D rigid-body model rather than a point with
  the following data:
  - 6 DOF target pose
  - Features attached
- Introduced 3 stochastic target motion models for a broad range of realistic
  target tracking scenarios

#### 2.1.4 Dense-ArthroSLAM: Dense Intra-Articular 3-D Reconstruction With Robust Localization Prior for Arthroscopy [@Marmol2019]

- [Paper](https://ieeexplore.ieee.org/document/8606964)

Features:

- Dense-ArthroSLAM: a SLAM system based on *ArthroSLAM* for arthroscopy 

- *ArthroSLAM*
  - A *sparse* SLAM system
  - 2 visual sensors are attached to the final link of a robotic arm
    - Arthroscopic camera for intra-articular tissues
    - External camera for the exterior of the patient anatomy
  - Raw camera measurements are fused with robot's odometry in an extended
    Kalman filter (EKF) framework
  - Role: as a robust localization prior for reliable camera pose information

- Improvement over original *ArthroSLAM*
  - Added a robust localizer with a keyframe selection strategy
    - A mix of 2 strategies:
      1) The amount of translational and rotational variation
      2) Certain number of frames
  - Added a batch multiview stereo (MVS) for 3D reconstruction
    - Re-detect SIFT features in the keyframes and do brute-force matching
    - Triangulation: 2 random keyframes having at least 5 degrees between 
      viewing rays
    - 3D points and camera poses are refined using BA for every keyframe added
      to the optimization
    - Outlier removal scheme
    - Use a patch-match for per-pixel intensity similarity, and jointly
      optimize depth and normal of point clouds
    - Valid pixel observations are fused into a dense scene reconstruction of
      the tissues
      - Resulting point cloud with associated normals can be used to infer a
        surface mesh and approximate a reconstructed area

- Outperform state-of-the-art SLAM methods (*ORB-SLAM* for feature-based,
  and *LSD-SLAM* for direct-based) both in reliability and estimation accuracy

System diagram:  
![System Diagram](images/ch06/marmol2019_fig_01.jpg){ width=50% }

- Art.: Arthroscopic
- Ext.: External
- New components are in red


\newpage
