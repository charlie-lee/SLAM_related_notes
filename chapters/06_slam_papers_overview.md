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
  1) Direct odometry module: use direct method to track new frames based on a 
     local semi-dense map
  2) Feature-based module: build a sparse feature-based map based on tracked 
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


\newpage
