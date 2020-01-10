# Overview of Various SLAM-related Papers

## 1. Classic Papers

### 1.1 Direct-Based Systems

#### 1.1.1 SVO - Fast Semi-Direct Monocular Visual Odometry [@Forster2014SVO]

- [Paper](https://infoscience.epfl.ch/record/199740/files/ICRA14_Forster.pdf)

##### 1.1.1.1 Overview

- System diagram:  
  ![System Diagram](images/ch06/forster2014_fig_01_diagram.jpg){ width=50% }

- System overview:

  - Motion estimation thread
    - Pose initialization: through sparse model-based image alignment
      - Find relative pose \( \V{T}_{k,k-1} \)
    - Pose & structure refinement: BA procedure
    
  - Mapping thread
    - Depth-filter
      - Initialized with a large uncertainty in depth
      - Updated in a Bayesian fashion
      - Depth is computed from the point in non-reference frame
        with the highest correlation (the point lying on the epipolar line) 
        with the reference patch
    - Add new 3D point only if the depth-filter is converged (i.e., depth
      uncertainty is small enough)
    
##### 1.1.1.2 Motion Estimation

- Procedures:

  1) Sparse Model-based Image Alignment  
     ![Sparse Alignment](images/ch06/forster2014_fig_02_sparse_img_align.jpg){ width=50% }
     - Goal: compute relative pose \( \V{T}_{k,k-1} \) from frame \( k-1 \) to
       frame \( k \) through minimization of photometric error of image patches
     - Minimization of reprojection error through Gauss-Newton procedure
     - Optimize both camera pose and image patch positions in the current frame
  
  2) Feature Alignment  
     ![Feature Alignment](images/ch06/forster2014_fig_03_feat_align.jpg){ width=50% }
     - Goal: further minimize photometric error of feature patches by estimating
       new feature positions in current frame w.r.t. reference keyframe
     - Solved using inverse compositional Lucas-Kanade algorithm [@Baker2004LK]
       - *Inverse compositional*: apply the delta parameters on the
         template/reference image during an iteration of optimization, instead
         of the current image, and then update the parameters with the inverse 
         of the delta parameters
     - Camera pose is not involved in the minimization scheme
     - Only the feature patch positions are optimized
     - A relaxation scheme: because of the violation of epipolar constraint
       for higher correlation between the feature patches
  
  3) Pose & Structure Refinement  
     ![BA](images/ch06/forster2014_fig_04_ba.jpg){ width=50% }
     - Step 1: motion-only BA: only optimize the camera pose
     - Step 2: structure-only BA: only optimize the 3D points
     - Step 3: (optional) local BA: optimize both local poses and 3D points
     - Minimization of reprojection error through Gauss-Newton procedure

- Advantages of the above procedures combined:

  - Advantage of the 1st procedure
    - The procedure implicitly satisfies epipolar constraint and ensures 
      there's no outliers
    - Without this procedure, the tracking will be slower because
      of the tracking of all features over large distances, whereas this
      procedure only optimizes 6 parameters
  - Advantage of the combination of the 3 procedures:
    - 2nd and 3rd procedures are necessary to reduce drift
      - 3rd procedure: image alignment w.r.t. the map and keyframes
      - 1st procedure only aligns image w.r.t. the previous frame

##### 1.1.1.3 Mapping

- Depth-filter in SVO:  
  ![Depth Filter](images/ch06/forster2014_fig_05_depth_filter.jpg){ width=50% }
  - Modification to the original depth-filter in [@Vogiatzis2011DF]

- Find depth of a image point via depth-filter associated with a keyframe
- Search for a patch on the epipolar line in the new image that has the 
  highest correlation (most probable depth) with the reference patch
- Depth measurement is modeled with a Gaussian + uniform mixture model
  - Gaussian distribution for inlier
  - Uniform distribution for outlier
  - Weighted average of the 2 distributions

##### 1.1.1.4 Implementation Details

- System initialization:
  - Assume a locally planar scene and estimate a homography: compute 
    1st pose via decomposition of \( \V{H} \)

- Sparse image alignment
  - Coarse-to-fine scheme: utilize a image pyramid of 5 levels with scale 
    factor of 2
  - Optimization on the coarsest level for an initial solution
  - Continue optimization on finer level and stop after convergence on the 
    3rd level

- Mapping
  - Fixed number of keyframes
    - Queue: new keyframe added will cause the farthest keyframe to be removed
  - Keyframe selection scheme: Euclidean distance of the new frame relative to
    all keyframes exceeds 12% of the average scene depth
  - Feature extraction
    - Divide the image in cells of fixed size (\( 30 \times 30 \) pixels)
    - Extract FAST corners with the highest Shi-Tomasi score in each image cell
      at *all levels* of the pyramid
      - Skip this step if the cell already has a 2D-to-3D correspondence

#### 1.1.2 LSD-SLAM: Large-scale Direct Monocular SLAM [@Engel2014LSD]

      
### 1.2 Feature-based Systems


### 1.3 Systems in Kalman Filter Framework


## 2. New Papers (2019)

### 2.1 IEEE International Conference on Robotics and Automation 2019 (ICRA2019)

#### 2.1.1 Loosely-Coupled Semi-Direct Monocular SLAM [@SHLee2019]

- [Paper](https://ieeexplore.ieee.org/abstract/document/8584894)

System diagram:  
![System Diagram](images/ch06/shlee2019_fig_02.jpg){ width=100% }
- Built upon direct sparse odometry (DSO) & ORB-SLAM

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

#### 2.1.2 Sparse2Dense: From Direct Sparse Odometry to Dense 3-D Reconstruction [@Tang2019]

- [Paper](https://ieeexplore.ieee.org/document/8605349)

System diagram:  
![System Diagram](images/ch06/tang2019_fig_02.jpg){ width=100% }

- 4 major stages:
  1) Depth/normal generation using CNN
  2) Visual tracking using direct alignment (direct sparse odometry, DSO)
  3) Geometrical sparse to dense reconstruction
  4) Fusion-based mapping
- Contribution on stage 1 & 3

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

#### 2.1.5 Modeling Perceptual Aliasing in SLAM via Discrete–Continuous Graphical Models [@Lajoie2019DCGM]

- [Paper](https://ieeexplore.ieee.org/document/8624393)

Features:

- A unified framework to model perceptual aliasing (wrong data association) in 
  SLAM 
  - Discrete-continuous graphical model (*DC-GM*)  
    ![DC-GM](images/ch06/lajoie2019_fig_01_dcgm.jpg){ width=60% }
    - Continuous portion: describes the standard SLAM formulation, i.e., 
      a pose graph
    - Discrete portion: describes outlier selection and models correlation
      - A Markov random field
- Practical algorithms to cope with outliers without relying on initial guess
  for optimization
  - Semidefinite relaxation to perform inference in *DC-GM* and returns 
    estimates with provable sub-optimality guarantees
    
Limitations:

- MATLAB implementation: slow, and only tested for relatively small 
  2D SLAM problems


### 2.2 Robotics: Science and Systems 2019 (RSS2019)

#### 2.2.1 Continuous Direct Sparse Visual Odometry from RGB-D Images [@Ghaffari2019RGBDVO]

- [Paper](http://www.roboticsproceedings.org/rss15/p44.html)

Features:

- A direct and continuous energy-based formulation and solution for the
  RGB-D visual odometry problem (sensor registration problem)
  - Direct: relates to direct-based method
  - Continuous: the formulation of the image registration problem, given
    discrete cloud measurements, is in a continuous form (based on continuous
    camera motion)
    - Operates from 3D space to an abstract information space such as 
      intensity surface
    - Continuity is achieved via functional treatment of the problem,
      and representing the functions in a reproducing kernel Hilbert space
- Do not rely on the association between 2 measurements and the same number
  of measurements within each set because of the continuous formulation
- The developed framework is not limited to specific image resolution
- The framework is fully analytical and the gradient has a complete closed-form
  derivation
  - It is opposed to the current direct energy formulation:
    a) Computation of numerical image intensity gradient
    b) The use of the gradient with the Jacobian of the pose via the chain rule
- The proposed method can be an alternative to the core module of many modern
  visual odometry and tracking systems

#### 2.2.2 PoseRBPF: A Rao-Blackwellized Particle Filter for 6D Object Pose Estimation [@Deng2019PoseRBPF]

- [Paper](http://www.roboticsproceedings.org/rss15/p49.html)

System diagram:  
![System Diagram](images/ch06/deng2019_fig_05_diagram.jpg){ width=100% }

- Propagation: estimate current pose based on previous pose using motion prior

Rotation computation:  
![Rotation Computation](images/ch06/deng2019_fig_03_rot.jpg){ width=100% }

Features:

- PoseRBPF: 6D object pose tracking based on Rao-Blackwellized particle 
  filter (RBPF) framework
- Pose is factorized into translation and rotation parts
  - Rotation
    - Rotation space is discretized at 5 degree resolution
      (\( 72 \times 37 \times 72 = 191808 \) bins for elevation only ranges
      from -90 to 90 degree)
    - An auto-encoder network is trained as an observation model to construct
      a codebook of feature embeddings for the discretized rotations
- Procedures: for each particle,
  1) Determine the center and the size of the object bounding box in the 
     image via 3D translation data
  2) Determine embedding of the bounding box
  3) Update rotation distribution by comparing the embedding value with the
     pre-computed entries in the codebook using cosine distance
  4) Compute the weight of the particle from the normalization factor of the
     rotation distribution
  5) Update motion by sampling from a motion model over poses and a 
     convolution over rotations
     
#### 2.2.3 VIMO: Simultaneous Visual Inertial Model-Based Odometry and Force Estimation [@Nisar2019VIMO]

Features:

- VIMO: visual inertial model-based odometry with an added motion constraint
  - Motion constraint is derived from the dynamic model including 
    *external forces*
  - Model dynamics and external force are combined in a preintegrated residual  
    ![Factor Graph](images/ch06/nisar2019_fig_01_graph.jpg){ width=60% }
    - The residual makes current visual inertial odometry (VIO) framework 
      jointly estimate this force in addition to the robot state and IMU bias
  - Built upon a state-of-the-art VIO system the VINS-mono system
  - Tightly-coupled, sliding-window visual inertial estimator
  - Input: visual-inertial measurements & the collective thrust
  - Estimation strategy: nonlinear optimization (BA)
- (1st approach to) Exploit model dynamics by jointly estimating motion 
  and external force

\newpage
